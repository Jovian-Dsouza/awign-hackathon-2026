import clsx from 'clsx'
import {
  BarChart3, Bot, ClipboardList, FlaskConical, LayoutDashboard, ListChecks, Menu, Monitor, Moon,
  Settings, ShieldCheck, Sun, X, type LucideIcon,
} from 'lucide-react'
import { useState } from 'react'
import { NavLink, Outlet } from 'react-router-dom'
import { api } from '../lib/api'
import { usePoll } from '../lib/hooks'
import { setThemeChoice, useThemeChoice, type ThemeChoice } from '../lib/theme'

const nav: { to: string; label: string; icon: LucideIcon }[] = [
  { to: '/', label: 'Overview', icon: LayoutDashboard },
  { to: '/robot', label: 'Live Robot', icon: Bot },
  { to: '/orders', label: 'Orders', icon: ClipboardList },
  { to: '/tasks', label: 'Tasks', icon: ListChecks },
  { to: '/qc', label: 'QC & Verification', icon: ShieldCheck },
  { to: '/analytics', label: 'Analytics', icon: BarChart3 },
  { to: '/settings', label: 'Settings', icon: Settings },
]

export function Logo() {
  return (
    <div className="flex items-center gap-2.5">
      <svg viewBox="0 0 32 32" className="size-8" aria-hidden>
        <rect width="32" height="32" rx="8" fill="#156cdd" />
        <path d="M9 8v9a7 7 0 0 0 14 0V8" fill="none" stroke="#ffffff" strokeWidth="3" strokeLinecap="round" />
        <circle cx="16" cy="17" r="2.5" fill="#ffffff" />
      </svg>
      <div className="leading-tight">
        <p className="text-[15px] font-semibold tracking-tight">UthaLe</p>
        <p className="text-[10px] text-ink-3">From Shelf to Dispatch. Intelligently.</p>
      </div>
    </div>
  )
}

function SystemStatus() {
  const { data: health, error } = usePoll(api.health, 4000)
  const { data: flagged } = usePoll(() => api.qc('flagged'), 4000)
  const up = !!health && !error
  return (
    <div className="space-y-2 rounded-xl border border-line bg-panel-2 p-3 text-xs">
      <Row label="Backend" ok={up} text={up ? 'Online' : 'Offline'} />
      <Row label="Camera" ok={health?.camera === 'streaming'} text={health ? health.camera : '—'} />
      <div className="flex items-center justify-between">
        <span className="text-ink-3">Robot</span>
        {health?.simulated
          ? <span className="inline-flex items-center gap-1 text-sim-ink"><FlaskConical className="size-3" /> Simulated</span>
          : <span className="text-ink-2">{health?.robot_adapter ?? '—'}</span>}
      </div>
      {!!flagged?.length && (
        <NavLink to="/qc" className="mt-1 flex items-center justify-between rounded-md border border-warn/30 bg-warn/10 px-2 py-1 text-warn-ink">
          <span>QC needs review</span><span className="tabular font-semibold">{flagged.length}</span>
        </NavLink>
      )}
    </div>
  )
}

const themeOptions: { value: ThemeChoice; label: string; icon: LucideIcon }[] = [
  { value: 'light', label: 'Light', icon: Sun },
  { value: 'dark', label: 'Dark', icon: Moon },
  { value: 'system', label: 'System', icon: Monitor },
]

function ThemeToggle() {
  const choice = useThemeChoice()
  return (
    <div className="grid grid-cols-3 gap-1 rounded-lg border border-line bg-panel-2 p-1" role="radiogroup" aria-label="Theme">
      {themeOptions.map(({ value, label, icon: Icon }) => (
        <button key={value} type="button" role="radio" aria-checked={choice === value} title={label}
          onClick={() => setThemeChoice(value)}
          className={clsx('inline-flex h-7 items-center justify-center gap-1 rounded-md text-[11px] font-medium transition-colors',
            choice === value ? 'bg-panel text-ink shadow-sm' : 'text-ink-3 hover:text-ink-2')}>
          <Icon className="size-3.5" aria-hidden />{label}
        </button>
      ))}
    </div>
  )
}

function Row({ label, ok, text }: { label: string; ok: boolean; text: string }) {
  return (
    <div className="flex items-center justify-between">
      <span className="text-ink-3">{label}</span>
      <span className={clsx('inline-flex items-center gap-1.5 capitalize', ok ? 'text-ok-ink' : 'text-crit-ink')}>
        <span className={clsx('size-1.5 rounded-full', ok ? 'bg-ok' : 'bg-crit')} aria-hidden />{text}
      </span>
    </div>
  )
}

function Sidebar({ onNavigate }: { onNavigate?: () => void }) {
  return (
    <div className="flex h-full flex-col gap-6 p-4">
      <div className="px-1 pt-1"><Logo /></div>
      <nav className="flex flex-col gap-0.5" aria-label="Main">
        {nav.map(({ to, label, icon: Icon }) => (
          <NavLink key={to} to={to} end={to === '/'} onClick={onNavigate}
            className={({ isActive }) => clsx(
              'group relative flex items-center gap-3 rounded-lg px-3 py-2 text-[13px] font-medium transition-colors',
              isActive ? 'bg-panel-3 text-ink' : 'text-ink-2 hover:bg-panel-2 hover:text-ink',
            )}>
            {({ isActive }) => (
              <>
                {isActive && <span className="absolute inset-y-1.5 left-0 w-0.5 rounded-full bg-brand" aria-hidden />}
                <Icon className={clsx('size-4', isActive ? 'text-brand' : 'text-ink-3 group-hover:text-ink-2')} aria-hidden />
                {label}
              </>
            )}
          </NavLink>
        ))}
      </nav>
      <div className="mt-auto space-y-3"><SystemStatus /><ThemeToggle /></div>
    </div>
  )
}

export function AppLayout() {
  const [open, setOpen] = useState(false)

  return (
    <div className="min-h-screen lg:grid lg:grid-cols-[248px_1fr]">
      <aside className="sticky top-0 hidden h-screen border-r border-line bg-panel/60 lg:block">
        <Sidebar />
      </aside>

      <div className="sticky top-0 z-30 flex items-center justify-between border-b border-line bg-ground/90 px-4 py-3 backdrop-blur lg:hidden">
        <Logo />
        <button type="button" className="rounded-lg border border-line p-2 text-ink-2" onClick={() => setOpen(true)} aria-label="Open menu">
          <Menu className="size-5" />
        </button>
      </div>
      {open && (
        <div className="fixed inset-0 z-40 bg-black/70 lg:hidden" onClick={() => setOpen(false)}>
          <aside className="h-full w-72 animate-fade-in border-r border-line bg-panel" onClick={(e) => e.stopPropagation()}>
            <button type="button" className="absolute right-3 top-3 rounded-lg p-2 text-ink-3" onClick={() => setOpen(false)} aria-label="Close menu">
              <X className="size-5" />
            </button>
            <Sidebar onNavigate={() => setOpen(false)} />
          </aside>
        </div>
      )}

      <main className="min-w-0 px-4 pb-10 pt-5 sm:px-6 lg:px-8 lg:pt-7">
        <Outlet />
      </main>
    </div>
  )
}

export function PageHeader({ title, description, actions }: { title: string; description?: React.ReactNode; actions?: React.ReactNode }) {
  return (
    <div className="mb-5 flex flex-wrap items-end justify-between gap-3">
      <div className="min-w-0">
        <h1 className="text-xl font-semibold tracking-tight">{title}</h1>
        {description && <p className="mt-1 text-[13px] text-ink-3">{description}</p>}
      </div>
      {actions && <div className="flex flex-wrap items-center gap-2">{actions}</div>}
    </div>
  )
}
