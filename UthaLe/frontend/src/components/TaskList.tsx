import clsx from 'clsx'
import { ChevronDown } from 'lucide-react'
import { useState } from 'react'
import { between, clock, duration } from '../lib/format'
import { taskIcons } from '../lib/taskIcons'
import type { Task } from '../types'
import { SimTag, TaskStatusBadge } from './ui'

/** Ordered task timeline for one order, with expandable execution history. */
export function TaskList({ tasks, activeId }: { tasks: Task[]; activeId?: string | null }) {
  const [open, setOpen] = useState<string | null>(null)
  return (
    <ol className="divide-y divide-line">
      {tasks.map((t, i) => {
        const Icon = taskIcons[t.type]
        const newLine = t.line_index !== null && (i === 0 || tasks[i - 1].line_index !== t.line_index)
        const expanded = open === t.id
        return (
          <li key={t.id} className={clsx(newLine && i > 0 && 'border-t-line-strong')}>
            <button type="button" onClick={() => setOpen(expanded ? null : t.id)} aria-expanded={expanded}
              className={clsx('flex w-full items-center gap-3 px-4 py-2.5 text-left transition-colors hover:bg-panel-2',
                t.id === activeId && 'bg-info/8')}>
              <span className={clsx('flex size-7 shrink-0 items-center justify-center rounded-full border',
                t.status === 'completed' ? 'border-ok/40 text-ok-ink'
                  : t.status === 'running' ? 'border-info/50 text-info-ink'
                    : t.status === 'failed' ? 'border-crit/50 text-crit-ink'
                      : t.status === 'awaiting_review' ? 'border-warn/50 text-warn-ink' : 'border-line-strong text-ink-3')}>
                <Icon className="size-3.5" aria-hidden />
              </span>
              <div className="min-w-0 flex-1">
                <p className="truncate text-[13px] text-ink">{t.title}</p>
                <p className="truncate text-[11px] text-ink-3">
                  <span className="font-mono">{t.id}</span>
                  {t.location && <> · {t.location}</>}
                  {t.attempts > 1 && <> · {t.attempts} attempts</>}
                  {t.error && <span className="text-crit-ink"> · {t.error}</span>}
                </p>
              </div>
              {t.source === 'simulated' && <span className="hidden sm:inline"><SimTag /></span>}
              <span className="hidden w-14 text-right text-[11px] text-ink-3 tabular sm:block">
                {duration(between(t.started_at, t.completed_at))}
              </span>
              <TaskStatusBadge status={t.status} />
              <ChevronDown className={clsx('size-4 shrink-0 text-ink-3 transition-transform', expanded && 'rotate-180')} />
            </button>
            {expanded && (
              <div className="bg-panel-2/60 px-4 py-3 pl-14">
                {t.events?.length ? (
                  <ol className="space-y-1.5 border-l border-line-strong pl-3">
                    {t.events.map((e, j) => (
                      <li key={j} className="text-xs">
                        <span className="font-mono text-ink-3 tabular">{clock(e.ts)}</span>{' '}
                        <span className="text-ink-2">{e.message}</span>
                      </li>
                    ))}
                  </ol>
                ) : <p className="text-xs text-ink-3">No execution history yet.</p>}
              </div>
            )}
          </li>
        )
      })}
    </ol>
  )
}
