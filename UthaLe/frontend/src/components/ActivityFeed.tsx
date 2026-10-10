import clsx from 'clsx'
import {
  Activity, AlertTriangle, Bot, Camera, CheckCircle2, ClipboardCheck, Cog, Hand, PackagePlus, Play,
  Route, ShieldCheck, Truck, UserCheck, XCircle, type LucideIcon,
} from 'lucide-react'
import { Link } from 'react-router-dom'
import { clock, timeAgo } from '../lib/format'
import { useNow } from '../lib/hooks'
import type { ActivityEvent } from '../types'
import { EmptyState, SimTag } from './ui'

const icons: Record<string, LucideIcon> = {
  order_created: PackagePlus,
  order_assigned: Bot,
  mission_created: Route,
  mission_started: Play,
  task_started: Play,
  item_picked: Hand,
  item_delivered: Truck,
  order_picked: ClipboardCheck,
  qc_pass: ShieldCheck,
  qc_flagged: AlertTriangle,
  exception: XCircle,
  review_completed: UserCheck,
  dispatched: CheckCircle2,
  robot_command: Bot,
  camera: Camera,
  system: Cog,
}

const sevClass = {
  info: 'text-ink-3 border-line-strong',
  success: 'text-ok-ink border-ok/40',
  warning: 'text-warn-ink border-warn/40',
  error: 'text-crit-ink border-crit/40',
}

export function ActivityFeed({ events, compact }: { events: ActivityEvent[] | null; compact?: boolean }) {
  const now = useNow(5000)
  if (!events) return <p className="p-4 text-xs text-ink-3">Loading activity…</p>
  if (events.length === 0) {
    return <EmptyState icon={Activity} title="No activity yet">Create an order to start the workflow.</EmptyState>
  }
  return (
    <ol className="divide-y divide-line">
      {events.map((e) => {
        const Icon = icons[e.type] ?? Activity
        return (
          <li key={e.id} className="flex animate-fade-in gap-3 px-4 py-2.5">
            <span className={clsx('mt-0.5 flex size-6 shrink-0 items-center justify-center rounded-md border bg-panel-2', sevClass[e.severity])}>
              <Icon className="size-3.5" aria-hidden />
            </span>
            <div className="min-w-0 flex-1">
              <p className={clsx('text-[13px] leading-snug text-ink', compact && 'line-clamp-2')}>{e.message}</p>
              <div className="mt-1 flex flex-wrap items-center gap-2 text-[11px] text-ink-3">
                <time className="tabular" dateTime={e.ts} title={new Date(e.ts).toLocaleString()}>{clock(e.ts)} · {timeAgo(e.ts, now)}</time>
                {e.order_id && <Link to={`/orders/${e.order_id}`} className="font-mono text-ink-2 hover:text-info-ink">{e.order_id}</Link>}
                {e.source === 'simulated' && <SimTag />}
                {e.source === 'user' && <span className="text-ink-3">· operator</span>}
              </div>
            </div>
          </li>
        )
      })}
    </ol>
  )
}
