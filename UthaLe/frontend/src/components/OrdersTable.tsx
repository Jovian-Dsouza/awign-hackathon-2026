import { AlertTriangle, PackageOpen } from 'lucide-react'
import { useNavigate } from 'react-router-dom'
import { clock, timeAgo, units } from '../lib/format'
import { useNow } from '../lib/hooks'
import type { Order } from '../types'
import { EmptyState, OrderStatusBadge, PriorityTag, ProgressBar } from './ui'

export function OrdersTable({ orders, empty }: { orders: Order[] | null; empty?: React.ReactNode }) {
  const navigate = useNavigate()
  const now = useNow(5000)
  if (!orders) return <p className="p-4 text-xs text-ink-3">Loading orders…</p>
  if (orders.length === 0) return <EmptyState icon={PackageOpen} title="No orders here">{empty}</EmptyState>
  return (
    <div className="overflow-x-auto">
      <table className="w-full min-w-[680px] text-left text-[13px]">
        <thead>
          <tr className="border-b border-line text-[11px] uppercase tracking-wider text-ink-3">
            <th className="px-4 py-2 font-medium">Order</th>
            <th className="px-3 py-2 font-medium">Items</th>
            <th className="px-3 py-2 font-medium">Priority</th>
            <th className="px-3 py-2 font-medium">Status</th>
            <th className="w-36 px-3 py-2 font-medium">Progress</th>
            <th className="px-3 py-2 font-medium">Robot</th>
            <th className="px-4 py-2 text-right font-medium">Created</th>
          </tr>
        </thead>
        <tbody className="divide-y divide-line">
          {orders.map((o) => (
            <tr key={o.id} tabIndex={0} onClick={() => navigate(`/orders/${o.id}`)}
              onKeyDown={(e) => e.key === 'Enter' && navigate(`/orders/${o.id}`)}
              className="cursor-pointer transition-colors hover:bg-panel-2 focus:bg-panel-2 focus:outline-none">
              <td className="px-4 py-2.5">
                <span className="font-mono text-ink">{o.id}</span>
                {o.flags > 0 && (
                  <span className="ml-2 inline-flex items-center gap-1 text-[11px] text-warn-ink" title="QC records need review">
                    <AlertTriangle className="size-3" />{o.flags}
                  </span>
                )}
              </td>
              <td className="px-3 py-2.5 text-ink-2 tabular">
                {o.items.length} line{o.items.length === 1 ? '' : 's'} · {units(o.items)} u
              </td>
              <td className="px-3 py-2.5"><PriorityTag priority={o.priority} /></td>
              <td className="px-3 py-2.5"><OrderStatusBadge status={o.status} /></td>
              <td className="px-3 py-2.5">
                <ProgressBar value={o.progress}
                  tone={o.status === 'exception' ? 'crit' : o.status === 'dispatched' ? 'ok' : o.flags ? 'warn' : 'info'} />
              </td>
              <td className="px-3 py-2.5 font-mono text-xs text-ink-2">{o.robot_id ?? <span className="text-ink-3">unassigned</span>}</td>
              <td className="px-4 py-2.5 text-right text-xs text-ink-2 tabular" title={new Date(o.created_at).toLocaleString()}>
                {clock(o.created_at)} <span className="text-ink-3">· {timeAgo(o.created_at, now)}</span>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}
