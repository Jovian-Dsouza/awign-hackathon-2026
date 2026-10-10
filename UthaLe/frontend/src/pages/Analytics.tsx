import { BarChart3, CheckCircle2, Clock, FlaskConical, Hand, Table2, XCircle, type LucideIcon } from 'lucide-react'
import { useState } from 'react'
import {
  Bar, BarChart, CartesianGrid, Legend, ResponsiveContainer, Tooltip, XAxis, YAxis,
} from 'recharts'
import { EmptyState, ErrorNote, Panel } from '../components/ui'
import { PageHeader } from '../layouts/AppLayout'
import { api } from '../lib/api'
import { duration, titleCase } from '../lib/format'
import { usePoll } from '../lib/hooks'
import type { Analytics } from '../types'

// Dark-mode steps of a validated categorical palette (fixed order) + status set.
const C = {
  s1: '#3987e5',
  s2: '#d95926',
  good: '#0ca30c',
  warning: '#fab219',
  critical: '#d03b3b',
  neutral: '#6f767e',
  grid: '#23282e',
  axis: '#2f363d',
  muted: '#898781',
  surface: '#111316',
}

const axisProps = {
  stroke: C.axis,
  tick: { fill: C.muted, fontSize: 11 },
  tickLine: false,
} as const

function ChartTooltip(props: { active?: boolean; label?: string | number; payload?: { name?: string; value?: number | string; color?: string }[]; fmt?: (v: number) => string }) {
  const { active, label, payload, fmt } = props
  if (!active || !payload?.length) return null
  return (
    <div className="rounded-lg border border-line-strong bg-panel-2 px-3 py-2 text-xs shadow-xl">
      {label !== undefined && <p className="mb-1 font-medium text-ink">{label}</p>}
      {payload.map((p) => (
        <p key={p.name} className="flex items-center gap-2 text-ink-2">
          <span className="size-2 rounded-sm" style={{ background: p.color }} aria-hidden />
          {p.name}: <span className="tabular text-ink">{fmt && typeof p.value === 'number' ? fmt(p.value) : p.value}</span>
        </p>
      ))}
    </div>
  )
}

function Stat({ label, value, sub, icon: Icon }: { label: string; value: string; sub?: string; icon: LucideIcon }) {
  return (
    <div className="rounded-xl border border-line bg-panel p-4">
      <div className="flex items-center justify-between">
        <span className="text-[11px] font-medium uppercase tracking-wider text-ink-3">{label}</span>
        <Icon className="size-4 text-ink-3" aria-hidden />
      </div>
      <p className="mt-3 text-2xl font-semibold tracking-tight">{value}</p>
      {sub && <p className="mt-1 text-[11px] text-ink-3">{sub}</p>}
    </div>
  )
}

const legendFmt = (v: string) => <span className="text-xs text-ink-2">{v}</span>

export function AnalyticsPage() {
  const { data, error } = usePoll(api.analytics, 5000)
  const [table, setTable] = useState(false)

  const tp = data?.throughput.points.map((p) => ({
    ...p,
    label: new Date(p.t).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
  })) ?? []
  const outcomes = data?.task_outcomes.map((t) => ({ ...t, name: titleCase(t.type) })) ?? []
  const durations = data?.fulfillment.recent ?? []
  const qc = data?.qc

  return (
    <>
      <PageHeader
        title="Analytics"
        description="Computed from persisted order, task and QC records. Nothing here is estimated or projected."
        actions={
          <button type="button" onClick={() => setTable(!table)}
            className="inline-flex h-9 items-center gap-1.5 rounded-lg border border-line-strong bg-panel-2 px-3 text-[13px] text-ink hover:bg-panel-3">
            {table ? <BarChart3 className="size-4" /> : <Table2 className="size-4" />}{table ? 'Charts' : 'Table view'}
          </button>
        }
      />
      {error && <div className="mb-4"><ErrorNote error={error} /></div>}
      {data?.data_source === 'simulated' && (
        <p className="mb-4 flex items-center gap-1.5 rounded-lg border border-sim/30 bg-sim/8 px-3 py-2 text-xs text-sim-ink">
          <FlaskConical className="size-3.5" /> Records come from the simulated robot workflow, not physical operations. Timings reflect simulator step durations.
        </p>
      )}

      <div className="grid grid-cols-2 gap-3 lg:grid-cols-4">
        <Stat icon={CheckCircle2} label="Orders dispatched" value={data ? `${data.totals.dispatched}` : '—'} sub={data ? `of ${data.totals.orders} created · ${data.totals.exceptions} exception(s)` : undefined} />
        <Stat icon={BarChart3} label="Task success rate" value={data?.task_success_rate != null ? `${(data.task_success_rate * 100).toFixed(1)}%` : '—'} sub={data ? `completed / (completed + failed) · n=${data.totals.tasks_finished}` : undefined} />
        <Stat icon={Clock} label="Avg fulfilment" value={duration(data?.fulfillment.avg_seconds)} sub={`median ${duration(data?.fulfillment.median_seconds)}`} />
        <Stat icon={Hand} label="QC human reviews" value={qc ? `${qc.approved + qc.rejected}` : '—'} sub={qc ? `${qc.approved} approved · ${qc.rejected} rejected` : undefined} />
      </div>

      {data && data.totals.orders === 0 ? (
        <div className="mt-4 rounded-xl border border-line bg-panel">
          <EmptyState icon={BarChart3} title="No data yet">Create and fulfil a few orders; charts fill in from real records.</EmptyState>
        </div>
      ) : table && data ? (
        <TableView data={data} />
      ) : (
        <div className="mt-4 grid gap-4 xl:grid-cols-2">
          <Panel title="Orders over time" subtitle={data?.throughput.bucket_minutes ? `${data.throughput.bucket_minutes}-minute buckets` : undefined}>
            <div className="h-64">
              <ResponsiveContainer>
                <BarChart data={tp} barGap={2} margin={{ top: 4, right: 4, left: -20, bottom: 0 }}>
                  <CartesianGrid stroke={C.grid} vertical={false} />
                  <XAxis dataKey="label" {...axisProps} minTickGap={16} />
                  <YAxis {...axisProps} axisLine={false} allowDecimals={false} />
                  <Tooltip cursor={{ fill: 'rgba(255,255,255,0.04)' }} content={<ChartTooltip />} />
                  <Legend formatter={legendFmt} iconType="square" iconSize={8} itemSorter={null} />
                  <Bar dataKey="created" name="Created" fill={C.s1} radius={[4, 4, 0, 0]} maxBarSize={18} />
                  <Bar dataKey="fulfilled" name="Dispatched" fill={C.s2} radius={[4, 4, 0, 0]} maxBarSize={18} />
                </BarChart>
              </ResponsiveContainer>
            </div>
          </Panel>

          <Panel title="Task outcomes by step" subtitle="Completed vs failed, awaiting review and skipped">
            <div className="h-64">
              <ResponsiveContainer>
                <BarChart data={outcomes} margin={{ top: 4, right: 4, left: -20, bottom: 0 }}>
                  <CartesianGrid stroke={C.grid} vertical={false} />
                  <XAxis dataKey="name" {...axisProps} />
                  <YAxis {...axisProps} axisLine={false} allowDecimals={false} />
                  <Tooltip cursor={{ fill: 'rgba(255,255,255,0.04)' }} content={<ChartTooltip />} />
                  <Legend formatter={legendFmt} iconType="square" iconSize={8} itemSorter={null} />
                  <Bar dataKey="completed" name="Completed" stackId="a" fill={C.good} stroke={C.surface} strokeWidth={2} maxBarSize={28} />
                  <Bar dataKey="awaiting_review" name="Awaiting review" stackId="a" fill={C.warning} stroke={C.surface} strokeWidth={2} maxBarSize={28} />
                  <Bar dataKey="failed" name="Failed" stackId="a" fill={C.critical} stroke={C.surface} strokeWidth={2} maxBarSize={28} />
                  <Bar dataKey="skipped" name="Skipped" stackId="a" fill={C.neutral} stroke={C.surface} strokeWidth={2} radius={[4, 4, 0, 0]} maxBarSize={28} />
                </BarChart>
              </ResponsiveContainer>
            </div>
          </Panel>

          <Panel title="Fulfilment duration" subtitle="Created → dispatched, last 20 orders (includes time waiting for review)">
            <div className="h-64">
              {durations.length === 0 ? <EmptyState icon={Clock} title="No dispatched orders yet" /> : (
                <ResponsiveContainer>
                  <BarChart data={durations} margin={{ top: 4, right: 4, left: -12, bottom: 0 }}>
                    <CartesianGrid stroke={C.grid} vertical={false} />
                    <XAxis dataKey="order_id" {...axisProps} minTickGap={8} />
                    <YAxis {...axisProps} axisLine={false} tickFormatter={(v: number) => duration(v)} />
                    <Tooltip cursor={{ fill: 'rgba(255,255,255,0.04)' }} content={<ChartTooltip fmt={duration} />} />
                    <Bar dataKey="seconds" name="Duration" fill={C.s1} radius={[4, 4, 0, 0]} maxBarSize={22} />
                  </BarChart>
                </ResponsiveContainer>
              )}
            </div>
          </Panel>

          <Panel title="QC results & exception categories">
            <div className="grid gap-4 md:grid-cols-[180px_1fr]">
              <ul className="space-y-2 text-[13px]">
                <QcRow icon={CheckCircle2} color={C.good} label="PASS" n={qc?.PASS} />
                <QcRow icon={Hand} color={C.warning} label="REVIEW" n={qc?.REVIEW} />
                <QcRow icon={XCircle} color={C.critical} label="FAIL" n={qc?.FAIL} />
              </ul>
              <div className="h-52">
                {!data?.exception_categories.length ? <EmptyState icon={CheckCircle2} title="No exceptions recorded" /> : (
                  <ResponsiveContainer>
                    <BarChart layout="vertical" data={data.exception_categories} margin={{ top: 0, right: 8, left: 0, bottom: 0 }}>
                      <CartesianGrid stroke={C.grid} horizontal={false} />
                      <XAxis type="number" {...axisProps} allowDecimals={false} />
                      <YAxis type="category" dataKey="category" {...axisProps} width={150} axisLine={false} />
                      <Tooltip cursor={{ fill: 'rgba(255,255,255,0.04)' }} content={<ChartTooltip />} />
                      <Bar dataKey="count" name="Occurrences" fill={C.s1} radius={[0, 4, 4, 0]} maxBarSize={16} />
                    </BarChart>
                  </ResponsiveContainer>
                )}
              </div>
            </div>
          </Panel>
        </div>
      )}
    </>
  )
}

function QcRow({ icon: Icon, color, label, n }: { icon: LucideIcon; color: string; label: string; n?: number }) {
  return (
    <li className="flex items-center justify-between rounded-lg border border-line bg-panel-2 px-3 py-2">
      <span className="inline-flex items-center gap-2 font-mono text-xs text-ink-2"><Icon className="size-4" style={{ color }} aria-hidden />{label}</span>
      <span className="text-lg font-semibold tabular">{n ?? '—'}</span>
    </li>
  )
}

function TableView({ data }: { data: Analytics }) {
  const th = 'px-3 py-2 text-left text-[11px] font-medium uppercase tracking-wider text-ink-3'
  const td = 'px-3 py-1.5 tabular text-ink-2'
  return (
    <div className="mt-4 grid gap-4 xl:grid-cols-2">
      <Panel title="Task outcomes" bodyClass="overflow-x-auto p-0">
        <table className="w-full text-xs">
          <thead><tr className="border-b border-line"><th className={th}>Step</th><th className={th}>Completed</th><th className={th}>Awaiting</th><th className={th}>Failed</th><th className={th}>Skipped</th></tr></thead>
          <tbody className="divide-y divide-line">
            {data.task_outcomes.map((t) => <tr key={t.type}><td className={td}>{titleCase(t.type)}</td><td className={td}>{t.completed}</td><td className={td}>{t.awaiting_review}</td><td className={td}>{t.failed}</td><td className={td}>{t.skipped}</td></tr>)}
          </tbody>
        </table>
      </Panel>
      <Panel title="Throughput" bodyClass="max-h-80 overflow-auto p-0">
        <table className="w-full text-xs">
          <thead><tr className="border-b border-line"><th className={th}>Bucket start</th><th className={th}>Created</th><th className={th}>Dispatched</th></tr></thead>
          <tbody className="divide-y divide-line">
            {data.throughput.points.map((p) => <tr key={p.t}><td className={td}>{new Date(p.t).toLocaleString()}</td><td className={td}>{p.created}</td><td className={td}>{p.fulfilled}</td></tr>)}
          </tbody>
        </table>
      </Panel>
      <Panel title="Fulfilment durations" bodyClass="max-h-80 overflow-auto p-0">
        <table className="w-full text-xs">
          <thead><tr className="border-b border-line"><th className={th}>Order</th><th className={th}>Duration</th></tr></thead>
          <tbody className="divide-y divide-line">
            {data.fulfillment.recent.map((d) => <tr key={d.order_id}><td className={`${td} font-mono`}>{d.order_id}</td><td className={td}>{duration(d.seconds)}</td></tr>)}
          </tbody>
        </table>
      </Panel>
      <Panel title="Exception categories" bodyClass="overflow-x-auto p-0">
        <table className="w-full text-xs">
          <thead><tr className="border-b border-line"><th className={th}>Category</th><th className={th}>Count</th></tr></thead>
          <tbody className="divide-y divide-line">
            {data.exception_categories.map((c) => <tr key={c.category}><td className={td}>{c.category}</td><td className={td}>{c.count}</td></tr>)}
          </tbody>
        </table>
      </Panel>
    </div>
  )
}
