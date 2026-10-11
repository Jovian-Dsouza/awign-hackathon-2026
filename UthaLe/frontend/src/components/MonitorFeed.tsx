import clsx from 'clsx'
import { AlertTriangle, Camera, CameraOff, Check, Crosshair, Eye, Loader2, RefreshCw, Square, X } from 'lucide-react'
import { useEffect, useRef, useState } from 'react'
import { api } from '../lib/api'
import { usePoll } from '../lib/hooks'
import type { MonitorIncident } from '../types'
import { Badge, Button, EmptyState, Panel } from './ui'

const SEVERITY = ['harmless', 'minor', 'major', 'critical']
const ACTION_LABEL: Record<string, string> = {
  ignore: 'False alarm', propose_recovery: 'Dropped item', escalate: 'Needs a person',
}

/**
 * External monitor camera: live annotated feed from the Overwatch service. It only raises an
 * incident when the robot drops something during a pickup. The admin decides: nothing here
 * moves the robot on its own.
 * If the monitor is not running the panel says so; it never shows placeholder footage.
 */
export function MonitorFeed() {
  const status = usePoll(api.monitorStatus, 2000)
  const reachable = status.data?.reachable === true
  const incidents = usePoll(() => (reachable ? api.monitorIncidents() : Promise.resolve([])), 2000, [reachable])
  const [nonce, setNonce] = useState(() => Date.now())
  const [failed, setFailed] = useState(false)
  const [note, setNote] = useState<string | null>(null)
  const [busy, setBusy] = useState<number | 'watch' | null>(null)

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

  async function startFromHere() {
    setBusy('watch')
    setNote(null)
    try {
      await api.monitorStart()
      await status.refresh()
    } catch (e) {
      setNote(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(null)
    }
  }

  async function stopMonitoring() {
    setBusy('watch')
    try {
      await api.monitorStop()
      await status.refresh()
    } catch (e) {
      setNote(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(null)
    }
  }

  const list = incidents.data ?? []
  const open = list.filter((i) => i.status === 'open')
  const warnings = status.data?.camera_warnings ?? []
  const watching = status.data?.watch?.watching === true
  const monitoring = status.data?.monitoring?.on === true
  const viewMoved = status.data?.monitoring?.view_moved === true

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
            <>
              <Button size="sm" variant={monitoring && !viewMoved ? 'secondary' : 'primary'} icon={Crosshair}
                onClick={startFromHere} disabled={busy === 'watch' || !streaming}
                title="Use the camera view and robot position as they are right now, and monitor from here">
                {monitoring ? 'Set new position' : 'Start monitoring from here'}
              </Button>
              {monitoring && (
                <Button size="sm" variant="ghost" icon={Square} onClick={stopMonitoring} disabled={busy === 'watch'}>
                  Stop
                </Button>
              )}
            </>
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
          {!monitoring ? (
            <Note tone="warn">
              Not monitoring yet. Put the camera and the robot where you want them, then press <b>Start monitoring from here</b>.
            </Note>
          ) : viewMoved ? (
            <Note tone="crit">The camera view moved. Press <b>Set new position</b> once it is where you want it.</Note>
          ) : (
            <Note tone={watching ? 'ok' : 'neutral'}>
              {watching
                ? <>Watching the robot. If it drops something you will see it here.</>
                : <>Monitoring from your position. Waiting for the robot to move.</>}
            </Note>
          )}
          {warnings.filter((w) => !viewMoved || !w.includes('Set the position again')).map((w) => <Note key={w} tone="warn">{w}</Note>)}
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
            <EmptyState icon={Check} title="No drops">
              Nothing dropped or spilled. If the robot drops something it shows up here with a picture and what to do.
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
  const isObject = inc.kind === 'dropped_item'
  const isSpill = inc.kind === 'spill'
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
          <Badge tone={inc.final.action === 'propose_recovery' ? 'info' : 'neutral'}>
            {isSpill ? 'Spill: needs a person' : inc.final.action === 'propose_recovery' ? 'Robot can pick it up' : ACTION_LABEL[inc.final.action]}
          </Badge>
          {inc.cleared_ts && <Badge tone="ok" icon={Check}>no longer visible</Badge>}
          {!open && <Badge tone="neutral">{inc.status}</Badge>}
        </div>
        <p className="mt-1 text-xs text-ink-2">{inc.observation.summary}</p>
        <p className="mt-0.5 text-[11px] text-ink-3">{inc.final.reasons.join(' ')}</p>
        {inc.laya && (
          <p className="mt-0.5 text-[11px] text-ink-3">
            Laya: {ACTION_LABEL[inc.laya.action] ?? inc.laya.action}
            {inc.laya.p_recoverable != null && ` · P(recoverable) ${Math.round(inc.laya.p_recoverable * 100)}%`}
            {inc.laya.latency_ms != null && ` · ${Math.round(inc.laya.latency_ms)} ms`}
          </p>
        )}
        {open ? (
          <div className="mt-2 flex flex-wrap gap-2">
            {isObject && (
              <Button size="sm" variant="primary" icon={Check} disabled={busy} onClick={() => onDecide(inc.id, 'approve_recovery')}>
                Ask the robot to pick it up
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

function Note({ tone, children }: { tone: 'warn' | 'crit' | 'info' | 'ok' | 'neutral'; children: React.ReactNode }) {
  return (
    <div className={clsx('rounded-xl border px-3 py-2 text-xs',
      tone === 'warn' && 'border-warn/30 bg-warn/10 text-warn-ink',
      tone === 'crit' && 'border-crit/30 bg-crit/10 text-crit-ink',
      tone === 'info' && 'border-info/30 bg-info/10 text-info-ink',
      tone === 'ok' && 'border-ok/30 bg-ok/10 text-ok-ink',
      tone === 'neutral' && 'border-line bg-panel-2 text-ink-2')}>
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
