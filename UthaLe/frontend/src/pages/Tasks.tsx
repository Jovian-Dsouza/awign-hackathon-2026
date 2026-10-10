import clsx from 'clsx'
import { ChevronDown, ListChecks } from 'lucide-react'
import { Fragment, useState } from 'react'
import { Link } from 'react-router-dom'
import { taskIcons } from '../lib/taskIcons'
import { EmptyState, ErrorNote, Panel, SimTag, Tabs, TaskStatusBadge } from '../components/ui'
import { PageHeader } from '../layouts/AppLayout'
import { api } from '../lib/api'
import { between, clock, duration, titleCase } from '../lib/format'
import { usePoll } from '../lib/hooks'
import type { Task } from '../types'

type Filter = 'all' | 'running' | 'awaiting_review' | 'failed' | 'completed'

export function TasksPage() {
  const [filter, setFilter] = useState<Filter>('all')
  const { data, error } = usePoll(() => api.tasks(), 1500)
  const [open, setOpen] = useState<string | null>(null)
  const detail = usePoll(() => (open ? api.task(open) : Promise.resolve(null)), 2000, [open])

  // Tasks that haven't started are noise in a global log; show them per order instead.
  const started = data?.filter((t) => t.status !== 'pending') ?? null
  const rows = started?.filter((t) => filter === 'all' || t.status === filter) ?? null
  const count = (s: Filter) => started?.filter((t) => s === 'all' || t.status === s).length

  return (
    <>
      <PageHeader title="Tasks" description="Individual robot steps and system checks, newest first. Pending steps appear on each order." />
      {error && <div className="mb-4"><ErrorNote error={error} /></div>}
      <div className="mb-3">
        <Tabs value={filter} onChange={setFilter} options={[
          { value: 'all', label: 'All', count: count('all') },
          { value: 'running', label: 'Running', count: count('running') },
          { value: 'awaiting_review', label: 'Awaiting review', count: count('awaiting_review') },
          { value: 'failed', label: 'Failed', count: count('failed') },
          { value: 'completed', label: 'Completed', count: count('completed') },
        ]} />
      </div>
      <Panel bodyClass="p-0">
        {!rows ? <p className="p-4 text-xs text-ink-3">Loading…</p> : rows.length === 0 ? (
          <EmptyState icon={ListChecks} title="No tasks match">Tasks are created when an order is placed and appear here once they start.</EmptyState>
        ) : (
          <div className="overflow-x-auto">
            <table className="w-full min-w-[760px] text-left text-[13px]">
              <thead>
                <tr className="border-b border-line text-[11px] uppercase tracking-wider text-ink-3">
                  <th className="px-4 py-2 font-medium">Task</th>
                  <th className="px-3 py-2 font-medium">Order</th>
                  <th className="px-3 py-2 font-medium">Type</th>
                  <th className="px-3 py-2 font-medium">Status</th>
                  <th className="px-3 py-2 text-right font-medium">Attempts</th>
                  <th className="px-3 py-2 text-right font-medium">Started</th>
                  <th className="px-3 py-2 text-right font-medium">Duration</th>
                  <th className="px-4 py-2 font-medium">Source</th>
                </tr>
              </thead>
              <tbody className="divide-y divide-line">
                {rows.map((t) => (
                  <Fragment key={t.id}>
                    <Row t={t} open={open === t.id} onToggle={() => setOpen(open === t.id ? null : t.id)} />
                    {open === t.id && (
                      <tr className="bg-panel-2/60">
                        <td colSpan={8} className="px-4 py-3 pl-12">
                          {t.error && <p className="mb-2 text-xs text-crit-ink">{t.error}</p>}
                          {detail.data?.events?.length ? (
                            <ol className="space-y-1.5 border-l border-line-strong pl-3">
                              {detail.data.events.map((e, i) => (
                                <li key={i} className="text-xs"><span className="font-mono text-ink-3 tabular">{clock(e.ts)}</span> <span className="text-ink-2">{e.message}</span></li>
                              ))}
                            </ol>
                          ) : <p className="text-xs text-ink-3">Loading history…</p>}
                        </td>
                      </tr>
                    )}
                  </Fragment>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </Panel>
    </>
  )
}

function Row({ t, open, onToggle }: { t: Task; open: boolean; onToggle: () => void }) {
  const Icon = taskIcons[t.type]
  return (
    <tr className="cursor-pointer hover:bg-panel-2" onClick={onToggle}>
      <td className="px-4 py-2.5">
        <div className="flex items-center gap-2">
          <ChevronDown className={clsx('size-3.5 shrink-0 text-ink-3 transition-transform', open && 'rotate-180')} />
          <div className="min-w-0">
            <p className="truncate text-ink">{t.title}</p>
            <p className="font-mono text-[11px] text-ink-3">{t.id}</p>
          </div>
        </div>
      </td>
      <td className="px-3 py-2.5"><Link onClick={(e) => e.stopPropagation()} to={`/orders/${t.order_id}`} className="font-mono text-xs text-info-ink hover:underline">{t.order_id}</Link></td>
      <td className="px-3 py-2.5"><span className="inline-flex items-center gap-1.5 text-xs text-ink-2"><Icon className="size-3.5 text-ink-3" />{titleCase(t.type)}</span></td>
      <td className="px-3 py-2.5"><TaskStatusBadge status={t.status} /></td>
      <td className="px-3 py-2.5 text-right text-xs tabular text-ink-2">{t.attempts}</td>
      <td className="px-3 py-2.5 text-right text-xs tabular text-ink-2">{clock(t.started_at)}</td>
      <td className="px-3 py-2.5 text-right text-xs tabular text-ink-2">{duration(between(t.started_at, t.completed_at))}</td>
      <td className="px-4 py-2.5">{t.source === 'simulated' ? <SimTag /> : <span className="text-xs text-ink-3">{t.source}</span>}</td>
    </tr>
  )
}
