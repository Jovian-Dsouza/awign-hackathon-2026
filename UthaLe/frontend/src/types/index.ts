export type Priority = 'low' | 'normal' | 'high'
export type OrderStatus = 'queued' | 'picking' | 'picked' | 'verification' | 'dispatched' | 'exception'
export type TaskStatus = 'pending' | 'running' | 'completed' | 'failed' | 'skipped' | 'awaiting_review'
export type TaskType = 'navigate' | 'identify' | 'pick' | 'transport' | 'place' | 'verify' | 'dispatch'
export type QcStatus = 'PASS' | 'FAIL' | 'REVIEW' | 'PENDING'
export type DataSource = 'simulated' | 'hardware' | 'system' | 'user' | 'none'
export type RobotMode = 'idle' | 'executing' | 'paused' | 'error' | 'stopped'
export type Severity = 'info' | 'success' | 'warning' | 'error'
export type Anomaly = 'weight_mismatch' | 'nfc_missing' | 'sku_mismatch' | 'grasp_failure'

export interface CatalogItem {
  sku: string
  name: string
  location: string
  shelf: string
  nominal_weight_g: number
}

export interface OrderLine {
  sku: string
  name: string
  quantity: number
  location: string
}

export interface Order {
  id: string
  items: OrderLine[]
  destination: string
  priority: Priority
  status: OrderStatus
  robot_id: string | null
  created_at: string
  updated_at: string
  started_at: string | null
  completed_at: string | null
  note: string | null
  progress: number
  flags: number
}

export interface TaskEvent {
  ts: string
  status: string
  message: string
}

export interface Task {
  id: string
  order_id: string
  seq: number
  line_index: number | null
  type: TaskType
  title: string
  sku: string | null
  location: string | null
  status: TaskStatus
  attempts: number
  source: DataSource
  created_at: string
  started_at: string | null
  completed_at: string | null
  error: string | null
  events?: TaskEvent[] | null
}

export interface SensorEvidence {
  nfc: { tag_read: boolean; tag_sku: string | null; source: string } | null
  load_cell: { delta_g: number; expected_g: number; source: string } | null
}

export interface QcRecord {
  id: string
  task_id: string
  order_id: string
  sku: string
  item_name: string
  expected_quantity: number
  expected_destination: string
  robot_outcome: string
  camera_evidence_url: string | null
  sensor_evidence: SensorEvidence
  evidence_source: DataSource
  status: QcStatus
  reasons: string[]
  review_decision: 'approved' | 'rejected' | null
  reviewer: string | null
  review_notes: string | null
  reviewed_at: string | null
  created_at: string
}

export interface OrderDetail extends Order {
  tasks: Task[]
  qc: QcRecord[]
}

export interface ActivityEvent {
  id: number
  ts: string
  type: string
  severity: Severity
  message: string
  order_id: string | null
  task_id: string | null
  source: DataSource
}

export interface RobotStatus {
  robot_id: string
  name: string
  model: string
  adapter: string
  simulated: boolean
  connected: boolean
  mode: RobotMode
  current_order_id: string | null
  current_task_id: string | null
  current_task_title: string | null
  zone: string | null
  battery_pct: number | null
  queue_length: number
  last_update: string
  stop_reason: string | null
  pending_anomaly: Anomaly | null
}

export interface CommandResult {
  accepted: boolean
  message: string
  simulated: boolean
  status: RobotStatus
}

export interface CameraStatus {
  state: 'streaming' | 'connecting' | 'error' | 'stopped'
  device_index: number
  backend: string
  width: number | null
  height: number | null
  fps: number | null
  frames: number
  last_frame_at: string | null
  error: string | null
  clients: number
}

export interface Overview {
  active_orders: number
  orders_fulfilled: number
  tasks_in_progress: number
  tasks_awaiting_verification: number
  failed_or_blocked_tasks: number
  avg_fulfillment_seconds: number | null
  fulfilled_sample_size: number
}

export interface Analytics {
  data_source: 'simulated' | 'recorded'
  totals: { orders: number; dispatched: number; exceptions: number; tasks_finished: number }
  throughput: { bucket_minutes: number | null; points: { t: string; created: number; fulfilled: number }[] }
  task_outcomes: { type: TaskType; completed: number; failed: number; awaiting_review: number; skipped: number }[]
  task_success_rate: number | null
  fulfillment: {
    recent: { order_id: string; completed_at: string; seconds: number }[]
    avg_seconds: number | null
    median_seconds: number | null
  }
  qc: { PASS: number; FAIL: number; REVIEW: number; PENDING: number; approved: number; rejected: number }
  exception_categories: { category: string; count: number }[]
}

export type SettingsInfo = Record<'camera' | 'robot' | 'sensors' | 'storage', Record<string, string | number | boolean | null>>

export interface Health {
  status: string
  time: string
  camera: string
  robot_adapter: string
  simulated: boolean
}


/** External monitor camera (Overwatch service), proxied by the backend at /api/monitor. */
export interface MonitorDecision {
  action: 'ignore' | 'propose_recovery' | 'escalate'
  severity: number
  recoverable: boolean
  reasons: string[]
  decider: string
  p_recoverable: number | null
  latency_ms: number | null
}

export interface MonitorIncident {
  id: number
  code: string
  ts: number
  kind: string
  zone: string
  status: 'open' | 'actioned' | 'escalated' | 'dismissed' | 'filtered'
  observation: { kind: string; zone: string; summary: string; features: Record<string, number | string | boolean> }
  final: MonitorDecision
  rules: MonitorDecision
  laya: MonitorDecision | null
  snapshot: string | null
  verdict: string | null
  cleared_ts: number | null
  responder: { ok: boolean; dry_run?: boolean; detail: string } | null
}

export interface MonitorStatus {
  reachable: boolean
  error?: string
  camera?: {
    state: string; source: string; width: number | null; height: number | null
    fps: number | null; error: string | null; agent_connected?: boolean
  }
  monitoring?: { on: boolean; since: number | null; view_moved: boolean }
  robot?: { in_view: boolean; tracking: 'tag' | 'motion'; tag_id: number | null }
  camera_warnings?: string[]
  watch?: { watching: boolean; why: string; seconds_left: number | null }
  ignored_changes?: { too_big: number; too_small: number; not_compact: number; outside_zone: number }
  alignment?: { shift_px?: number; match?: number }
  health?: { active_faults: string[] }
  laya?: { mode: string; available: boolean; error: string | null } | null
  responder?: { name: string; dry_run: boolean; reachable: boolean | null }
  open_incidents?: number
}
