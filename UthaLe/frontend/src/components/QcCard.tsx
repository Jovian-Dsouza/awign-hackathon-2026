import clsx from 'clsx'
import { Camera, Check, ImageOff, Nfc, Scale, UserCheck, X } from 'lucide-react'
import { useState } from 'react'
import { Link } from 'react-router-dom'
import { api } from '../lib/api'
import { dateTime, titleCase } from '../lib/format'
import type { QcRecord } from '../types'
import { Badge, Button, ErrorNote, QcBadge, SimTag, inputClass } from './ui'

const REVIEWER_KEY = 'uthale.reviewer'

function loadReviewer() {
  try {
    return localStorage.getItem(REVIEWER_KEY) ?? ''
  } catch {
    return ''
  }
}

export function QcCard({ rec, onReviewed, showOrder = true }: { rec: QcRecord; onReviewed?: () => void; showOrder?: boolean }) {
  const [reviewer, setReviewer] = useState(loadReviewer)
  const [notes, setNotes] = useState('')
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [zoom, setZoom] = useState(false)
  const needsReview = (rec.status === 'FAIL' || rec.status === 'REVIEW') && !rec.review_decision
  const sim = rec.evidence_source === 'simulated'
  const nfc = rec.sensor_evidence.nfc
  const load = rec.sensor_evidence.load_cell

  async function decide(decision: 'approved' | 'rejected') {
    if (!reviewer.trim()) {
      setError('Enter your name to sign the review.')
      return
    }
    setBusy(true)
    setError(null)
    try {
      try {
        localStorage.setItem(REVIEWER_KEY, reviewer.trim())
      } catch { /* storage unavailable */ }
      await api.review(rec.task_id, { decision, reviewer: reviewer.trim(), notes: notes.trim() || undefined })
      onReviewed?.()
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(false)
    }
  }

  return (
    <article className={clsx('@container animate-fade-in rounded-xl border bg-panel',
      needsReview ? (rec.status === 'FAIL' ? 'border-crit/45' : 'border-warn/45') : 'border-line')}>
      <header className="flex flex-wrap items-center gap-2 border-b border-line px-4 py-3">
        <QcBadge status={rec.status} />
        <h3 className="text-[13px] font-semibold">{rec.item_name}<span className="font-normal text-ink-3"> ×{rec.expected_quantity}</span></h3>
        <span className="font-mono text-[11px] text-ink-3">{rec.id}</span>
        <div className="ml-auto flex items-center gap-2">
          {sim && <SimTag label="SIM SENSORS" title="NFC and load-cell readings are simulated" />}
          {rec.review_decision && (
            <Badge tone={rec.review_decision === 'approved' ? 'ok' : 'crit'} icon={UserCheck}>
              {titleCase(rec.review_decision)} · {rec.reviewer}
            </Badge>
          )}
        </div>
      </header>

      <div className="grid gap-4 p-4 @xl:grid-cols-[200px_1fr]">
        <div>
          {rec.camera_evidence_url ? (
            <button type="button" onClick={() => setZoom(true)} className="group relative block w-full overflow-hidden rounded-lg border border-line bg-black">
              <img src={rec.camera_evidence_url} alt={`Camera frame at verification of ${rec.id}`} className="aspect-video w-full object-cover transition-opacity group-hover:opacity-80" loading="lazy" />
              <span className="absolute left-1.5 top-1.5 inline-flex items-center gap-1 rounded bg-black/70 px-1.5 py-0.5 text-[10px] text-white"><Camera className="size-3" />frame</span>
            </button>
          ) : (
            <div className="flex aspect-video w-full flex-col items-center justify-center gap-1 rounded-lg border border-dashed border-line-strong text-ink-3">
              <ImageOff className="size-4" /><span className="text-[11px]">No camera evidence</span>
            </div>
          )}
          <p className="mt-1.5 text-[10px] leading-snug text-ink-3">Real camera frame captured at verification time. Shows the scene; not an automated judgment.</p>
        </div>

        <div className="min-w-0 space-y-3">
          <dl className="grid grid-cols-2 gap-x-4 gap-y-2 text-xs @3xl:grid-cols-4">
            <Item label="Expected SKU"><span className="font-mono">{rec.sku}</span></Item>
            <Item label="Destination">{rec.expected_destination}</Item>
            <Item label="Robot reported">
              <span className={rec.robot_outcome === 'placed' ? 'text-ink' : 'text-crit-ink'}>{titleCase(rec.robot_outcome)}</span>
            </Item>
            {showOrder && <Item label="Order"><Link className="font-mono text-info-ink hover:underline" to={`/orders/${rec.order_id}`}>{rec.order_id}</Link></Item>}
          </dl>

          <div className="grid gap-2 @md:grid-cols-2">
            <Sensor icon={Nfc} label="NFC tag" sim={nfc?.source === 'simulated'}
              value={!nfc ? 'Not connected' : nfc.tag_read ? nfc.tag_sku ?? '—' : 'No tag read'}
              ok={!!nfc && nfc.tag_read && nfc.tag_sku === rec.sku} missing={!nfc} />
            <Sensor icon={Scale} label="Load cell Δ" sim={load?.source === 'simulated'}
              value={load ? `+${load.delta_g.toFixed(0)} g / ≈${load.expected_g} g` : 'Not connected'}
              ok={!!load && Math.abs(load.delta_g - load.expected_g) <= Math.max(25, 0.1 * load.expected_g)} missing={!load} />
          </div>

          {rec.reasons.length > 0 && (
            <ul className="space-y-1 text-xs">
              {rec.reasons.map((r) => (
                <li key={r} className={clsx('flex gap-2', rec.status === 'FAIL' ? 'text-crit-ink' : 'text-warn-ink')}>
                  <span aria-hidden>•</span>{r}
                </li>
              ))}
            </ul>
          )}

          {rec.review_decision && (
            <p className="text-xs text-ink-3">
              Reviewed {dateTime(rec.reviewed_at)}{rec.review_notes ? `: “${rec.review_notes}”` : ''}
            </p>
          )}

          {needsReview && (
            <div className="space-y-2 rounded-lg border border-line bg-panel-2 p-3">
              <p className="text-xs font-medium text-ink-2">Human review required</p>
              <div className="grid gap-2 @md:grid-cols-[160px_1fr]">
                <input className={inputClass} placeholder="Reviewer name" value={reviewer} onChange={(e) => setReviewer(e.target.value)} maxLength={60} aria-label="Reviewer name" />
                <input className={inputClass} placeholder="Notes (e.g. checked tote by hand)" value={notes} onChange={(e) => setNotes(e.target.value)} maxLength={500} aria-label="Review notes" />
              </div>
              <ErrorNote error={error} />
              <div className="flex flex-wrap gap-2">
                <Button size="sm" variant="primary" icon={Check} disabled={busy} onClick={() => decide('approved')}>Approve, item correct</Button>
                <Button size="sm" variant="danger" icon={X} disabled={busy} onClick={() => decide('rejected')}>Reject, hold order</Button>
              </div>
            </div>
          )}
        </div>
      </div>

      {zoom && rec.camera_evidence_url && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/85 p-6" onClick={() => setZoom(false)}>
          <img src={rec.camera_evidence_url} alt="Camera evidence" className="max-h-full max-w-full rounded-lg" />
        </div>
      )}
    </article>
  )
}

function Item({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="min-w-0">
      <dt className="text-[10px] uppercase tracking-wider text-ink-3">{label}</dt>
      <dd className="mt-0.5 truncate text-ink">{children}</dd>
    </div>
  )
}

function Sensor({ icon: Icon, label, value, ok, missing, sim }: {
  icon: typeof Nfc; label: string; value: string; ok: boolean; missing: boolean; sim: boolean
}) {
  return (
    <div className={clsx('flex items-center gap-2.5 rounded-lg border px-3 py-2',
      missing ? 'border-line border-dashed' : ok ? 'border-ok/30 bg-ok/5' : 'border-warn/40 bg-warn/5')}>
      <Icon className={clsx('size-4 shrink-0', missing ? 'text-ink-3' : ok ? 'text-ok-ink' : 'text-warn-ink')} />
      <div className="min-w-0 flex-1">
        <p className="text-[10px] uppercase tracking-wider text-ink-3">{label}{sim && <span className="text-sim-ink"> · sim</span>}</p>
        <p className="truncate font-mono text-xs text-ink">{value}</p>
      </div>
      {!missing && (ok ? <Check className="size-3.5 text-ok-ink" aria-label="matches" /> : <X className="size-3.5 text-warn-ink" aria-label="does not match" />)}
    </div>
  )
}
