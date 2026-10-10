import { FlaskConical, ShieldCheck } from 'lucide-react'
import { useState } from 'react'
import { QcCard } from '../components/QcCard'
import { EmptyState, ErrorNote, Tabs } from '../components/ui'
import { PageHeader } from '../layouts/AppLayout'
import { api } from '../lib/api'
import { usePoll } from '../lib/hooks'

type Filter = 'flagged' | 'all' | 'PASS' | 'reviewed'

export function QcPage() {
  const [filter, setFilter] = useState<Filter>('flagged')
  const { data, error, refresh } = usePoll(() => api.qc(), 2000)
  const flagged = data?.filter((q) => (q.status === 'FAIL' || q.status === 'REVIEW') && !q.review_decision)
  const rows = data?.filter((q) =>
    filter === 'all' ? true
      : filter === 'flagged' ? flagged?.includes(q)
        : filter === 'reviewed' ? !!q.review_decision
          : q.status === filter)

  return (
    <>
      <PageHeader
        title="QC & verification"
        description="Each placed item is checked against independent evidence. A robot saying “placed” is never enough for a PASS."
      />
      {error && <div className="mb-4"><ErrorNote error={error} /></div>}

      <div className="mb-4 grid gap-3 rounded-xl border border-line bg-panel p-4 text-xs text-ink-2 md:grid-cols-3">
        <div><p className="font-medium text-ink">PASS</p><p className="mt-0.5 text-ink-3">Robot placed the item, the NFC tag matches the SKU, and the tote weight is within ±10%.</p></div>
        <div><p className="font-medium text-ink">REVIEW</p><p className="mt-0.5 text-ink-3">Evidence is missing or unclear (no tag read, weight off, no sensors). A human decides.</p></div>
        <div><p className="font-medium text-ink">FAIL</p><p className="mt-0.5 text-ink-3">Evidence contradicts the order (wrong item, empty tote, pick failed). Human review required.</p></div>
        <p className="flex items-center gap-1.5 text-sim-ink md:col-span-3">
          <FlaskConical className="size-3.5" /> NFC and load-cell sensors are not connected yet. Readings are simulated and labelled SIM. Camera frames are real.
        </p>
      </div>

      <div className="mb-3">
        <Tabs value={filter} onChange={setFilter} options={[
          { value: 'flagged', label: 'Needs review', count: flagged?.length },
          { value: 'reviewed', label: 'Reviewed', count: data?.filter((q) => q.review_decision).length },
          { value: 'PASS', label: 'Passed', count: data?.filter((q) => q.status === 'PASS').length },
          { value: 'all', label: 'All', count: data?.length },
        ]} />
      </div>

      {!rows ? <p className="text-xs text-ink-3">Loading…</p> : rows.length === 0 ? (
        <div className="rounded-xl border border-line bg-panel">
          <EmptyState icon={ShieldCheck} title={filter === 'flagged' ? 'Nothing waiting for review' : 'No records'}>
            {filter === 'flagged' ? 'Flagged placements show up here. To demo one, arm an anomaly on the Live Robot page.' : 'Records are created as the robot places items.'}
          </EmptyState>
        </div>
      ) : (
        <div className="grid gap-3 2xl:grid-cols-2">
          {rows.map((q) => <QcCard key={q.id} rec={q} onReviewed={refresh} />)}
        </div>
      )}
    </>
  )
}
