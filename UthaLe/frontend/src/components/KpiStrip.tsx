import clsx from 'clsx'
import { AlertOctagon, CheckCircle2, Hand, Loader, PackageOpen, Timer, type LucideIcon } from 'lucide-react'
import { duration } from '../lib/format'
import type { Overview } from '../types'

function Kpi({ label, value, icon: Icon, hint, tone }: {
  label: string
  value: string | number
  icon: LucideIcon
  hint?: string
  tone?: 'warn' | 'crit' | 'ok'
}) {
  return (
    <div className="flex min-w-0 flex-col justify-between gap-3 rounded-xl border border-line bg-panel p-4 shadow-[0_1px_2px_rgba(15,23,42,0.04)]">
      <div className="flex items-center justify-between gap-2">
        <span className="truncate text-[11px] font-medium uppercase tracking-wider text-ink-3">{label}</span>
        <Icon className={clsx('size-4 shrink-0',
          tone === 'crit' ? 'text-crit-ink' : tone === 'warn' ? 'text-warn-ink' : tone === 'ok' ? 'text-ok-ink' : 'text-ink-3')} aria-hidden />
      </div>
      <div>
        <div className="text-2xl font-semibold leading-none tracking-tight text-ink">{value}</div>
        {hint && <div className="mt-1.5 truncate text-[11px] text-ink-3">{hint}</div>}
      </div>
    </div>
  )
}

export function KpiStrip({ data }: { data: Overview | null }) {
  const d = data
  return (
    <div className="grid grid-cols-2 gap-3 sm:grid-cols-3 xl:grid-cols-6">
      <Kpi label="Active orders" icon={PackageOpen} value={d?.active_orders ?? '—'} hint="queued → verification" />
      <Kpi label="Orders fulfilled" icon={CheckCircle2} value={d?.orders_fulfilled ?? '—'} hint="dispatched" tone={d?.orders_fulfilled ? 'ok' : undefined} />
      <Kpi label="Tasks in progress" icon={Loader} value={d?.tasks_in_progress ?? '—'} hint="robot steps running" />
      <Kpi label="Awaiting verification" icon={Hand} value={d?.tasks_awaiting_verification ?? '—'}
        hint="needs human review" tone={d?.tasks_awaiting_verification ? 'warn' : undefined} />
      <Kpi label="Failed / blocked" icon={AlertOctagon} value={d?.failed_or_blocked_tasks ?? '—'}
        hint="on open orders" tone={d?.failed_or_blocked_tasks ? 'crit' : undefined} />
      <Kpi label="Avg fulfillment" icon={Timer}
        value={d?.avg_fulfillment_seconds != null ? duration(d.avg_fulfillment_seconds) : '—'}
        hint={d?.fulfilled_sample_size ? `created → dispatched · n=${d.fulfilled_sample_size}` : 'no completed orders yet'} />
    </div>
  )
}
