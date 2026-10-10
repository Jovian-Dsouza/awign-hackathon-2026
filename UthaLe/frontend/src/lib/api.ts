import type {
  ActivityEvent, Analytics, Anomaly, CameraStatus, CatalogItem, CommandResult, Health, Order,
  OrderDetail, Overview, Priority, QcRecord, RobotStatus, SettingsInfo, Task,
} from '../types'

/** Empty = same origin (the Vite dev server proxies /api to FastAPI). */
export const API_BASE: string = (import.meta.env.VITE_API_URL as string | undefined) ?? ''

export class ApiError extends Error {
  status: number
  constructor(status: number, message: string) {
    super(message)
    this.status = status
  }
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  let res: Response
  try {
    res = await fetch(`${API_BASE}${path}`, {
      ...init,
      headers: { 'Content-Type': 'application/json', ...(init?.headers ?? {}) },
    })
  } catch {
    throw new ApiError(0, 'Backend unreachable. Is the API running on port 8000?')
  }
  if (!res.ok) {
    let detail = res.statusText
    try {
      const body = await res.json()
      detail = typeof body.detail === 'string' ? body.detail : JSON.stringify(body.detail)
    } catch {
      /* non-JSON error body */
    }
    throw new ApiError(res.status, detail)
  }
  return res.json() as Promise<T>
}

const post = <T>(path: string, body?: unknown) =>
  request<T>(path, { method: 'POST', body: body === undefined ? undefined : JSON.stringify(body) })

export const api = {
  health: () => request<Health>('/api/health'),
  overview: () => request<Overview>('/api/overview'),
  analytics: () => request<Analytics>('/api/analytics'),
  settings: () => request<SettingsInfo>('/api/settings'),

  cameraStatus: () => request<CameraStatus>('/api/camera/status'),
  cameraStreamUrl: (nonce: number) => `${API_BASE}/api/camera/stream?n=${nonce}`,
  cameraStart: () => post<CameraStatus>('/api/camera/start'),
  cameraStop: () => post<CameraStatus>('/api/camera/stop'),
  cameraSetIndex: (device_index: number) =>
    request<CameraStatus>('/api/camera/config', { method: 'PUT', body: JSON.stringify({ device_index }) }),

  robotStatus: () => request<RobotStatus>('/api/robot/status'),
  robotCommand: (command: string, extra: { order_id?: string; anomaly?: Anomaly } = {}) =>
    post<CommandResult>('/api/robot/commands', { command, ...extra }),

  catalog: () => request<CatalogItem[]>('/api/catalog'),
  destinations: () => request<string[]>('/api/destinations'),

  orders: (status?: string) => request<Order[]>(`/api/orders${status ? `?status=${status}` : ''}`),
  order: (id: string) => request<OrderDetail>(`/api/orders/${encodeURIComponent(id)}`),
  createOrder: (body: {
    items: { sku: string; quantity: number }[]
    destination: string
    priority: Priority
    assign_robot: boolean
    note?: string
  }) => post<Order>('/api/orders', body),
  createDemoOrder: () => post<Order>('/api/orders/demo'),
  assignOrder: (id: string, robot_id: string) =>
    post<Order>(`/api/orders/${encodeURIComponent(id)}/assign`, { robot_id }),

  tasks: (params: { status?: string; order_id?: string } = {}) => {
    const q = new URLSearchParams(Object.entries(params).filter(([, v]) => v) as [string, string][])
    return request<Task[]>(`/api/tasks${q.size ? `?${q}` : ''}`)
  },
  task: (id: string) => request<Task>(`/api/tasks/${encodeURIComponent(id)}`),

  qc: (status?: string) => request<QcRecord[]>(`/api/qc${status ? `?status=${status}` : ''}`),
  review: (taskId: string, body: { decision: 'approved' | 'rejected'; reviewer: string; notes?: string }) =>
    post<QcRecord>(`/api/qc/${encodeURIComponent(taskId)}/review`, body),

  activity: (limit = 40, orderId?: string) =>
    request<ActivityEvent[]>(`/api/activity?limit=${limit}${orderId ? `&order_id=${orderId}` : ''}`),
}
