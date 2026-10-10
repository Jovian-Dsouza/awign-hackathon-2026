import clsx from 'clsx'
import {
  AlertTriangle, CheckCircle2, CircleDashed, CircleDot, Clock, FlaskConical, Hand, Loader2,
  type LucideIcon, MinusCircle, PauseCircle, Truck, XCircle,
} from 'lucide-react'
import type { ReactNode } from 'react'
import type { OrderStatus, Priority, QcStatus, RobotMode, TaskStatus } from '../types'

export function Panel({ title, icon: Icon, actions, children, className, bodyClass, subtitle }: {
  title?: ReactNode
  subtitle?: ReactNode
  icon?: LucideIcon
  actions?: ReactNode
  children: ReactNode
  className?: string
  bodyClass?: string
}) {
  return (
    <section className={clsx('flex min-w-0 flex-col glass rounded-2xl border border-glass-edge', className)}>
      {title && (
        <header className="flex flex-wrap items-center gap-x-3 gap-y-2 border-b border-line px-4 py-3">
          {Icon && <Icon className="size-4 text-ink-3" aria-hidden />}
          <div className="min-w-0 flex-1">
            <h2 className="truncate text-[13px] font-semibold tracking-wide text-ink">{title}</h2>
            {subtitle && <p className="truncate text-xs text-ink-3">{subtitle}</p>}
          </div>
          {actions && <div className="flex flex-wrap items-center gap-2">{actions}</div>}
        </header>
      )}
      <div className={clsx('min-w-0 flex-1', bodyClass ?? 'p-4')}>{children}</div>
    </section>
  )
}

type Tone = 'ok' | 'warn' | 'crit' | 'info' | 'sim' | 'neutral' | 'serious'

const toneClass: Record<Tone, string> = {
  ok: 'border-ok/35 bg-ok/12 text-ok-ink',
  warn: 'border-warn/35 bg-warn/10 text-warn-ink',
  serious: 'border-serious/35 bg-serious/10 text-serious',
  crit: 'border-crit/40 bg-crit/12 text-crit-ink',
  info: 'border-info/35 bg-info/10 text-info-ink',
  sim: 'border-sim/40 bg-sim/12 text-sim-ink',
  neutral: 'border-line bg-panel-2 text-ink-2',
}

export function Badge({ tone = 'neutral', icon: Icon, children, className, spin }: {
  tone?: Tone
  icon?: LucideIcon
  children: ReactNode
  className?: string
  spin?: boolean
}) {
  return (
    <span className={clsx('inline-flex items-center gap-1 whitespace-nowrap rounded-full border px-2 py-0.5 text-[11px] font-medium', toneClass[tone], className)}>
      {Icon && <Icon className={clsx('size-3', spin && 'animate-spin')} aria-hidden />}
      {children}
    </span>
  )
}

/** Marks a value that comes from simulation, not hardware. */
export function SimTag({ label = 'SIM', title }: { label?: string; title?: string }) {
  return (
    <Badge tone="sim" icon={FlaskConical} className="font-mono tracking-wider">
      <span title={title ?? 'Simulated: not real robot or sensor telemetry'}>{label}</span>
    </Badge>
  )
}

const orderStatus: Record<OrderStatus, { tone: Tone; icon: LucideIcon; label: string; spin?: boolean }> = {
  queued: { tone: 'neutral', icon: Clock, label: 'Queued' },
  picking: { tone: 'info', icon: Loader2, label: 'Picking', spin: true },
  picked: { tone: 'info', icon: CheckCircle2, label: 'Picked' },
  verification: { tone: 'warn', icon: Hand, label: 'Verification' },
  dispatched: { tone: 'ok', icon: Truck, label: 'Dispatched' },
  exception: { tone: 'crit', icon: XCircle, label: 'Exception' },
}

export function OrderStatusBadge({ status }: { status: OrderStatus }) {
  const s = orderStatus[status]
  return <Badge tone={s.tone} icon={s.icon} spin={s.spin}>{s.label}</Badge>
}

const taskStatus: Record<TaskStatus, { tone: Tone; icon: LucideIcon; label: string; spin?: boolean }> = {
  pending: { tone: 'neutral', icon: CircleDashed, label: 'Pending' },
  running: { tone: 'info', icon: Loader2, label: 'Running', spin: true },
  completed: { tone: 'ok', icon: CheckCircle2, label: 'Completed' },
  failed: { tone: 'crit', icon: XCircle, label: 'Failed' },
  skipped: { tone: 'neutral', icon: MinusCircle, label: 'Skipped' },
  awaiting_review: { tone: 'warn', icon: Hand, label: 'Awaiting review' },
}

export function TaskStatusBadge({ status }: { status: TaskStatus }) {
  const s = taskStatus[status]
  return <Badge tone={s.tone} icon={s.icon} spin={s.spin}>{s.label}</Badge>
}

const qcStatus: Record<QcStatus, { tone: Tone; icon: LucideIcon }> = {
  PASS: { tone: 'ok', icon: CheckCircle2 },
  FAIL: { tone: 'crit', icon: XCircle },
  REVIEW: { tone: 'warn', icon: AlertTriangle },
  PENDING: { tone: 'neutral', icon: CircleDashed },
}

export function QcBadge({ status }: { status: QcStatus }) {
  const s = qcStatus[status]
  return <Badge tone={s.tone} icon={s.icon} className="font-mono tracking-wide">{status}</Badge>
}

const modeStyle: Record<RobotMode, { tone: Tone; icon: LucideIcon; label: string }> = {
  idle: { tone: 'neutral', icon: CircleDot, label: 'Idle' },
  executing: { tone: 'ok', icon: Loader2, label: 'Executing' },
  paused: { tone: 'warn', icon: PauseCircle, label: 'Paused' },
  error: { tone: 'crit', icon: XCircle, label: 'Error' },
  stopped: { tone: 'crit', icon: XCircle, label: 'Stopped' },
}

export function ModeBadge({ mode }: { mode: RobotMode }) {
  const s = modeStyle[mode]
  return <Badge tone={s.tone} icon={s.icon} spin={mode === 'executing'}>{s.label}</Badge>
}

export function PriorityTag({ priority }: { priority: Priority }) {
  return (
    <span className={clsx('inline-flex items-center gap-1.5 text-xs capitalize',
      priority === 'high' ? 'text-ink' : priority === 'low' ? 'text-ink-3' : 'text-ink-2')}>
      <span className={clsx('inline-block h-2.5 w-1 rounded-sm',
        priority === 'high' ? 'bg-serious' : priority === 'low' ? 'bg-line-strong' : 'bg-ink-3')} aria-hidden />
      {priority}
    </span>
  )
}

export function ProgressBar({ value, tone = 'info' }: { value: number; tone?: 'info' | 'ok' | 'warn' | 'crit' }) {
  const color = { info: 'bg-info', ok: 'bg-ok', warn: 'bg-warn', crit: 'bg-crit' }[tone]
  return (
    <div className="h-1.5 w-full overflow-hidden rounded-full bg-panel-3" role="progressbar"
      aria-valuenow={Math.round(value * 100)} aria-valuemin={0} aria-valuemax={100}>
      <div className={clsx('h-full rounded-full transition-[width] duration-500', color)} style={{ width: `${Math.round(value * 100)}%` }} />
    </div>
  )
}

export function Button({ variant = 'secondary', size = 'md', icon: Icon, children, className, ...rest }: {
  variant?: 'primary' | 'secondary' | 'ghost' | 'danger' | 'warn'
  size?: 'sm' | 'md'
  icon?: LucideIcon
} & React.ButtonHTMLAttributes<HTMLButtonElement>) {
  return (
    <button
      type="button"
      {...rest}
      className={clsx(
        'inline-flex items-center justify-center gap-1.5 rounded-full border font-medium transition-all active:scale-[0.97] disabled:cursor-not-allowed disabled:opacity-45 disabled:active:scale-100',
        size === 'sm' ? 'h-7 px-3 text-xs' : 'h-9 px-4 text-[13px]',
        {
          primary: 'border-transparent bg-brand text-white shadow-[0_4px_14px_-4px_rgba(21,108,221,0.6)] hover:bg-brand/90',
          secondary: 'border-transparent bg-panel-2 text-ink hover:bg-panel-3',
          ghost: 'border-transparent text-ink-2 hover:bg-panel-2 hover:text-ink',
          danger: 'border-transparent bg-crit/14 text-crit-ink hover:bg-crit/22',
          warn: 'border-transparent bg-warn/16 text-warn-ink hover:bg-warn/24',
        }[variant],
        className,
      )}
    >
      {Icon && <Icon className="size-4" aria-hidden />}
      {children}
    </button>
  )
}

export function EmptyState({ icon: Icon, title, children }: { icon: LucideIcon; title: string; children?: ReactNode }) {
  return (
    <div className="flex flex-col items-center justify-center gap-2 px-6 py-10 text-center">
      <div className="rounded-full bg-panel-2 p-3"><Icon className="size-5 text-ink-3" /></div>
      <p className="text-sm font-medium text-ink-2">{title}</p>
      {children && <div className="max-w-sm text-xs text-ink-3">{children}</div>}
    </div>
  )
}

export function ErrorNote({ error }: { error: string | null }) {
  if (!error) return null
  return (
    <div className="flex items-center gap-2 rounded-xl border border-crit/30 bg-crit/10 px-3 py-2 text-xs text-crit-ink">
      <AlertTriangle className="size-4 shrink-0" /> {error}
    </div>
  )
}

export function Field({ label, children }: { label: ReactNode; children: ReactNode }) {
  return (
    <div className="min-w-0">
      <dt className="text-[11px] font-medium uppercase tracking-wider text-ink-3">{label}</dt>
      <dd className="mt-1 truncate text-[13px] text-ink">{children}</dd>
    </div>
  )
}

export function Tabs<T extends string>({ value, onChange, options }: {
  value: T
  onChange: (v: T) => void
  options: { value: T; label: string; count?: number }[]
}) {
  return (
    <div className="inline-flex flex-wrap gap-0.5 rounded-full bg-panel-3 p-0.5" role="tablist">
      {options.map((o) => (
        <button key={o.value} type="button" role="tab" aria-selected={value === o.value} onClick={() => onChange(o.value)}
          className={clsx('inline-flex h-7 items-center gap-1.5 rounded-full px-3 text-xs font-medium transition-all',
            value === o.value ? 'bg-elevated text-ink shadow-[0_1px_3px_rgba(0,0,0,0.12)]' : 'text-ink-2 hover:text-ink')}>
          {o.label}
          {o.count != null && <span className="tabular rounded-full bg-panel-2 px-1.5 text-[10px] text-ink-2">{o.count}</span>}
        </button>
      ))}
    </div>
  )
}

export const inputClass =
  'h-9 w-full rounded-xl border border-transparent bg-panel-2 px-3 text-[13px] text-ink placeholder:text-ink-3 transition-shadow focus:border-brand/50 focus:outline-none focus:ring-4 focus:ring-brand/15'
