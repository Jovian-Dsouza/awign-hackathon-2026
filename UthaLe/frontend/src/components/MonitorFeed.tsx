import clsx from 'clsx'
import { AlertTriangle, Camera, CameraOff, Check, Eye, Loader2, RefreshCw, X } from 'lucide-react'
import { useEffect, useRef, useState } from 'react'
import { api } from '../lib/api'
import { usePoll } from '../lib/hooks'
import type { MonitorIncident } from '../types'
import { Badge, Button, EmptyState, Panel } from './ui'

const SEVERITY = ['harmless', 'minor', 'major', 'critical']
const ACTION_LABEL: Record<string, string> = {
  ignore: 'Ignore', propose_recovery: 'Recovery proposed', escalate: 'Needs a person',
}

/**
 * External monitor camera: live annotated feed from the Overwatch service, plus the
 * incidents it raised. The admin decides: nothing here moves the robot on its own.
 * If the monitor is not running the panel says so; it never shows placeholder footage.
 */
export function MonitorFeed() {
  const status = usePoll(api.monitorStatus, 2000)
  const reachable = status.data?.reachable === true
  const incidents = usePoll(() => (reachable ? api.monitorIncidents() : Promise.resolve([])), 2000, [reachable])
  const [nonce, setNonce] = useState(() => Date.now())
  const [failed, setFailed] = useState(false)
  const [note, setNote] = useState<string | null>(null)
  const [busy, setBusy] = useState<number | 'ref' | null>(null)

  const cam = status.data?.camera
  const streaming = reachable && cam?.state === 'streaming'
  const wasStreaming = useRef(false)
  useEffect(() => {
    if (streaming && !wasStreaming.current) {
      setFailed(false)
      setNonce(Date.now())
    }
    wasStreaming.current = streaming
  }, [streaming])

  async function decide(id: number, verdict: 'approve_recovery' | 'escalate' | 'dismiss') {
    setBusy(id)
    setNote(null)
    try {
      const r = await api.monitorVerdict(id, verdict)
      if (r.responder && !r.responder.ok) setNote(`Robot could not act: ${r.responder.detail}`)
      else if (r.responder?.dry_run) setNote(r.responder.detail)
      await incidents.refresh()
      await status.refresh()
    } catch (e) {
      setNote(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(null)
    }
  }

  async function captureReference() {
    setBusy('ref')
    setNote(null)
    try {
      await api.monitorCaptureReference()
      await status.refresh()
    } catch (e) {
      setNote(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(null)
    }
  }

  const list = incidents.data ?? []
  const open = list.filter((i) => i.status === 'open')
  const faults = status.data?.health?.active_faults ?? []
  const moved = !!status.data?.reference_set && ((status.data?.alignment?.shift_px ?? 0) > 2.5)

  return (
    <Panel
      icon={Eye}
      title="External monitor"
      subtitle={!status.data ? 'Connecting to monitor service…'
        : !reachable ? 'Monitor service not reachable'
          : `${cam?.source ?? 'camera'}${cam?.width ? ` · ${cam.width}×${cam.height}` : ''}${cam?.fps ? ` · ${cam.fps} fps` : ''}`}
      bodyClass="p-0"
      actions={
        <>
          <StateBadge reachable={reachable} state={cam?.state} loading={!status.data} />
          {reachable && (
            <Button size="sm" icon={RefreshCw} onClick={captureReference} disabled={busy === 'ref' || !streaming}
              title="Use the current view as the clean reference (arena tidy, robot out of view)">
              {status.data?.reference_set ? 'New reference' : 'Capture reference'}
            </Button>
          )}
        </>
      }
    >
      <div className="relative aspect-video min-h-[200px] w-full overflow-hidden bg-black">
        {streaming && !failed ? (
          <MjpegImage src={api.monitorStreamUrl(nonce)} onError={() => setFailed(true)} />
        ) : (
          <Offline
            reachable={reachable}
            message={status.error ?? status.data?.error ?? cam?.error ?? (failed ? 'The monitor stream dropped.' : null)}
            onRetry={() => { setFailed(false); setNonce(Date.now()); status.refresh() }}
          />
        )}
      </div>

      {reachable && (
        <div className="space-y-2 border-t border-line px-4 py-3">
          {!status.data?.reference_set && (
            <Note tone="warn">No clean reference yet, so object detection is idle. Tidy the arena, take the robot out of view and press <b>Capture reference</b>.</Note>
          )}
          {moved && <Note tone="crit">The camera view has moved since the reference was captured. Detection is paused until it is put back or a new reference is captured.</Note>}
          {faults.filter((f) => f !== 'scene_shift').map((f) => (
            <Note key={f} tone="crit">Camera fault: {f.replace(/_/g, ' ')}. The monitor cannot see mistakes while this lasts.</Note>
          ))}
          {note && <Note tone="info">{note}</Note>}
          {status.data?.laya && (
            <p className="text-[11px] text-ink-3">
              Laya decision model: <b>{status.data.laya.mode}</b>{status.data.laya.available ? '' : ' · not loaded'}
              {status.data.responder?.dry_run ? ' · robot responder: dry run (nothing is sent to the Pi)' : ''}
            </p>
          )}
        </div>
      )}

      {reachable && (
        <div className="border-t border-line">
          {open.length === 0 && list.length === 0 ? (
            <EmptyState icon={Check} title="No incidents">
              The monitor has not flagged anything. Incidents appear here with a snapshot and what to do about them.
            </EmptyState>
          ) : (
            <ul className="divide-y divide-line">
              {[...open, ...list.filter((i) => i.status !== 'open')].slice(0, 6).map((i) => (
                <Incident key={i.id} inc={i} busy={busy === i.id} onDecide={decide} />
              ))}
            </ul>
          )}
        </div>
      )}
    </Panel>
  )
}

function Incident({ inc, busy, onDecide }: {
  inc: MonitorIncident
  busy: boolean
  onDecide: (id: number, v: 'approve_recovery' | 'escalate' | 'dismiss') => void
}) {
  const open = inc.status === 'open'
  const isObject = inc.kind === 'foreign_object'
  return (
    <li className={clsx('flex gap-3 px-4 py-3', !open && 'opacity-70')}>
      {inc.snapshot ? (
        <img src={api.monitorSnapshotUrl(inc.id)} alt={`${inc.code} snapshot`} loading="lazy"
          className="h-20 w-28 shrink-0 rounded-lg bg-black object-cover" />
      ) : <div className="h-20 w-28 shrink-0 rounded-lg bg-panel-2" />}
      <div className="min-w-0 flex-1">
        <div className="flex flex-wrap items-center gap-1.5">
          <span className="text-[13px] font-semibold">{inc.code}</span>
          <Badge tone={inc.final.severity >= 2 ? 'crit' : 'warn'} icon={AlertTriangle}>{SEVERITY[inc.final.severity]}</Badge>
          <Badge tone={inc.final.action === 'propose_recovery' ? 'info' : 'neutral'}>{ACTION_LABEL[inc.final.action]}</Badge>
          {inc.cleared_ts && <Badge tone="ok" icon={Check}>no longer visible</Badge>}
          {!open && <Badge tone="neutral">{inc.status}</Badge>}
        </div>
        <p className="mt-1 text-xs text-ink-2">{inc.observation.summary}</p>
        <p className="mt-0.5 text-[11px] text-ink-3">{inc.final.reasons.join(' ')}</p>
        {inc.laya && (
          <p className="mt-0.5 text-[11px] text-ink-3">
            Laya (shadow): {ACTION_LABEL[inc.laya.action] ?? inc.laya.action}
            {inc.laya.p_recoverable != null && ` · P(recoverable) ${Math.round(inc.laya.p_recoverable * 100)}%`}
            {inc.laya.latency_ms != null && ` · ${Math.round(inc.laya.latency_ms)} ms`}
          </p>
        )}
        {open ? (
          <div className="mt-2 flex flex-wrap gap-2">
            {isObject && (
              <Button size="sm" variant="primary" icon={Check} disabled={busy} onClick={() => onDecide(inc.id, 'approve_recovery')}>
                Approve recovery
              </Button>
            )}
            <Button size="sm" variant="warn" icon={AlertTriangle} disabled={busy} onClick={() => onDecide(inc.id, 'escalate')}>
              Needs a person
            </Button>
            <Button size="sm" variant="ghost" icon={X} disabled={busy} onClick={() => onDecide(inc.id, 'dismiss')}>
              False alarm
            </Button>
          </div>
        ) : inc.responder ? (
          <p className="mt-1 text-[11px] text-ink-3">Robot: {inc.responder.detail}</p>
        ) : null}
      </div>
    </li>
  )
}

function Note({ tone, children }: { tone: 'warn' | 'crit' | 'info'; children: React.ReactNode }) {
  return (
    <div className={clsx('rounded-xl border px-3 py-2 text-xs',
      tone === 'warn' && 'border-warn/30 bg-warn/10 text-warn-ink',
      tone === 'crit' && 'border-crit/30 bg-crit/10 text-crit-ink',
      tone === 'info' && 'border-info/30 bg-info/10 text-info-ink')}>
      {children}
    </div>
  )
}

function StateBadge({ reachable, state, loading }: { reachable: boolean; state?: string; loading: boolean }) {
  if (loading) return <Badge tone="info" icon={Loader2} spin>Connecting</Badge>
  if (!reachable) return <Badge tone="crit" icon={CameraOff}>Monitor offline</Badge>
  if (state === 'streaming') return <Badge tone="ok" icon={Camera}>Streaming</Badge>
  if (state === 'waiting') return <Badge tone="warn" icon={Loader2} spin>Waiting for camera</Badge>
  return <Badge tone="crit" icon={CameraOff}>No signal</Badge>
}

function Offline({ reachable, message, onRetry }: { reachable: boolean; message: string | null; onRetry: () => void }) {
  return (
    <div className="grid-texture absolute inset-0 flex items-center justify-center bg-panel-2/80 p-4">
      <div className="glass-strong max-w-md rounded-2xl border border-glass-edge p-4 text-left">
        <div className="flex items-center gap-2">
          <CameraOff className="size-4 text-crit-ink" />
          <p className="text-sm font-semibold">{reachable ? 'No monitor camera signal' : 'Monitor service not running'}</p>
        </div>
        {message && <p className="mt-2 text-xs text-ink-2">{message}</p>}
        {!reachable && (
          <ul className="mt-3 list-disc space-y-1 pl-4 text-xs text-ink-3">
            <li>Start it: <code className="font-mono text-ink-2">cd overwatch &amp;&amp; uv run overwatch</code></li>
            <li>Remote camera: run <code className="font-mono text-ink-2">camera-agent --via-uthale</code> on the device the camera is plugged into.</li>
          </ul>
        )}
        <p className="mt-3 text-[11px] text-ink-3">No placeholder footage is shown in place of a missing camera.</p>
        <Button size="sm" className="mt-3" icon={RefreshCw} onClick={onRetry}>Retry</Button>
      </div>
    </div>
  )
}

/** Clearing `src` on unmount aborts the MJPEG request; leaked streams otherwise exhaust the
 *  browser's per-host connection limit (same reasoning as CameraFeed). */
function MjpegImage({ src, onError }: { src: string; onError: () => void }) {
  const ref = useRef<HTMLImageElement>(null)
  useEffect(() => {
    const el = ref.current
    if (!el) return
    el.src = src
    return () => { el.src = 'data:,' }
  }, [src])
  return <img ref={ref} alt="External monitor camera feed" className="h-full w-full object-contain" onError={onError} />
}
