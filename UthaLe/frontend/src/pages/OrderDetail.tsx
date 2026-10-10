import clsx from 'clsx'
import { Activity, ArrowLeft, Bot, Check, ListChecks, Package, Play, ShieldCheck } from 'lucide-react'
import { useState } from 'react'
import { Link, useParams } from 'react-router-dom'
import { ActivityFeed } from '../components/ActivityFeed'
import { QcCard } from '../components/QcCard'
import { TaskList } from '../components/TaskList'
import { Button, EmptyState, ErrorNote, Field, OrderStatusBadge, Panel, PriorityTag, ProgressBar } from '../components/ui'
import { api } from '../lib/api'
import { between, dateTime, duration, units } from '../lib/format'
import { usePoll } from '../lib/hooks'
import type { OrderStatus } from '../types'

const STEPS: { key: OrderStatus; label: string }[] = [
  { key: 'queued', label: 'Queued' },
  { key: 'picking', label: 'Picking' },
  { key: 'picked', label: 'Picked' },
  { key: 'verification', label: 'Verification' },
  { key: 'dispatched', label: 'Dispatched' },
]

function Stepper({ status }: { status: OrderStatus }) {
  const idx = status === 'exception' ? 3 : STEPS.findIndex((s) => s.key === status)
  return (
    <ol className="flex items-center">
      {STEPS.map((s, i) => {
        const done = i < idx || status === 'dispatched'
        const current = i === idx && status !== 'dispatched'
        const failed = status === 'exception' && i === idx
        return (
          <li key={s.key} className="flex flex-1 items-center last:flex-none">
            <div className="flex flex-col items-center gap-1.5">
              <span className={clsx('flex size-6 items-center justify-center rounded-full border text-[11px] font-semibold',
                failed ? 'border-crit bg-crit/20 text-crit-ink'
                  : done ? 'border-ok bg-ok/20 text-ok-ink'
                    : current ? 'border-info bg-info/20 text-info-ink' : 'border-line-strong text-ink-3')}>
                {done ? <Check className="size-3.5" /> : i + 1}
              </span>
              <span className={clsx('whitespace-nowrap text-[11px]', current || failed ? 'text-ink' : 'text-ink-3')}>
                {failed ? 'Exception' : s.label}
              </span>
            </div>
            {i < STEPS.length - 1 && <span className={clsx('mx-1 mb-5 h-px flex-1', i < idx || status === 'dispatched' ? 'bg-ok/60' : 'bg-line-strong')} />}
          </li>
        )
      })}
    </ol>
  )
}

export function OrderDetailPage() {
  const { orderId = '' } = useParams()
  const { data: o, error, refresh } = usePoll(() => api.order(orderId), 1200, [orderId])
  const activity = usePoll(() => api.activity(60, orderId), 2000, [orderId])
  const [busy, setBusy] = useState(false)
  const [actionError, setActionError] = useState<string | null>(null)

  async function act(fn: () => Promise<unknown>) {
    setBusy(true)
    setActionError(null)
    try {
      await fn()
      refresh()
    } catch (e) {
      setActionError(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(false)
    }
  }

  if (!o) {
    return (
      <div className="space-y-4">
        <Link to="/orders" className="inline-flex items-center gap-1 text-xs text-ink-3 hover:text-ink"><ArrowLeft className="size-3.5" /> Orders</Link>
        {error ? <ErrorNote error={error} /> : <p className="text-xs text-ink-3">Loading order…</p>}
      </div>
    )
  }

  const lines = o.items.map((line, idx) => ({
    line,
    tasks: o.tasks.filter((t) => t.line_index === idx),
  }))

  return (
    <>
      <Link to="/orders" className="mb-3 inline-flex items-center gap-1 text-xs text-ink-3 hover:text-ink"><ArrowLeft className="size-3.5" /> Orders</Link>
      <div className="mb-5 flex flex-wrap items-center gap-3">
        <h1 className="font-mono text-xl font-semibold tracking-tight">{o.id}</h1>
        <OrderStatusBadge status={o.status} />
        <PriorityTag priority={o.priority} />
        <div className="ml-auto flex flex-wrap gap-2">
          {o.status === 'queued' && !o.robot_id && (
            <Button icon={Bot} disabled={busy} onClick={() => act(() => api.assignOrder(o.id, 'SIM-STRETCH-01'))}>Assign to SIM-STRETCH-01</Button>
          )}
          {o.status === 'queued' && (
            <Button variant="primary" icon={Play} disabled={busy} onClick={() => act(() => api.robotCommand('start', { order_id: o.id }))}>Start now</Button>
          )}
        </div>
      </div>
      {actionError && <div className="mb-4"><ErrorNote error={actionError} /></div>}

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-12">
        <div className="min-w-0 space-y-4 xl:col-span-8">
          <Panel>
            <Stepper status={o.status} />
            <div className="mt-5"><ProgressBar value={o.progress} tone={o.status === 'exception' ? 'crit' : o.status === 'dispatched' ? 'ok' : o.flags ? 'warn' : 'info'} /></div>
            <dl className="mt-5 grid grid-cols-2 gap-x-6 gap-y-4 sm:grid-cols-4">
              <Field label="Dispatch zone">{o.destination}</Field>
              <Field label="Robot"><span className="font-mono">{o.robot_id ?? 'Unassigned'}</span></Field>
              <Field label="Created">{dateTime(o.created_at)}</Field>
              <Field label="Fulfilment time">{o.completed_at ? duration(between(o.created_at, o.completed_at)) : '—'}</Field>
              <Field label="Units">{units(o.items)}</Field>
              <Field label="Started">{dateTime(o.started_at)}</Field>
              <Field label="Completed">{dateTime(o.completed_at)}</Field>
              <Field label="Note">{o.note ?? '—'}</Field>
            </dl>
          </Panel>

          <Panel icon={ListChecks} title="Tasks" subtitle="Click a task for its execution history" bodyClass="p-0">
            {lines.map(({ line, tasks }, i) => (
              <div key={i} className="border-b border-line last:border-b-0">
                <div className="flex items-center gap-2 bg-panel-2/50 px-4 py-2 text-xs">
                  <Package className="size-3.5 text-ink-3" />
                  <span className="font-medium text-ink">{line.name}</span>
                  <span className="text-ink-3">×{line.quantity} · {line.location}</span>
                </div>
                <TaskList tasks={tasks} />
              </div>
            ))}
            <TaskList tasks={o.tasks.filter((t) => t.line_index === null)} />
          </Panel>
        </div>

        <div className="min-w-0 space-y-4 xl:col-span-4">
          <Panel icon={ShieldCheck} title="Verification" subtitle={`${o.qc.length} of ${o.items.length} line(s) checked`} bodyClass="space-y-3 p-3">
            {o.qc.length === 0
              ? <EmptyState icon={ShieldCheck} title="No QC records yet">Each line is verified after the robot places it.</EmptyState>
              : o.qc.map((q) => <QcCard key={q.id} rec={q} onReviewed={refresh} showOrder={false} />)}
          </Panel>
          <Panel icon={Activity} title="Order activity" bodyClass="max-h-[420px] overflow-y-auto p-0">
            <ActivityFeed events={activity.data} compact />
          </Panel>
        </div>
      </div>
    </>
  )
}
