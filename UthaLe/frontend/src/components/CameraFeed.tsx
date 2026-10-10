import clsx from 'clsx'
import { Camera, CameraOff, Loader2, Maximize2, Play, RefreshCw, Square } from 'lucide-react'
import { useEffect, useRef, useState } from 'react'
import { api } from '../lib/api'
import { clock } from '../lib/format'
import { useNow, usePoll } from '../lib/hooks'
import { Badge, Button, Panel } from './ui'

/**
 * Live USB camera panel. Shows the backend MJPEG stream only while the backend
 * reports real frames. When the camera is unavailable it says so; it never
 * substitutes placeholder footage.
 */
export function CameraFeed({ className, tall }: { className?: string; tall?: boolean }) {
  const { data: status, error: apiError, refresh } = usePoll(api.cameraStatus, 2000)
  const [nonce, setNonce] = useState(() => Date.now())
  const [imgFailed, setImgFailed] = useState(false)
  const [busy, setBusy] = useState(false)
  const frameRef = useRef<HTMLDivElement>(null)
  const now = useNow(1000)

  const streaming = status?.state === 'streaming'
  const showStream = streaming && !imgFailed && !apiError

  // When the backend recovers (reconnect, index change), reopen the <img> stream.
  const wasStreaming = useRef(false)
  useEffect(() => {
    if (streaming && !wasStreaming.current) {
      setImgFailed(false)
      setNonce(Date.now())
    }
    wasStreaming.current = streaming
  }, [streaming])

  async function toggle() {
    setBusy(true)
    try {
      if (streaming || status?.state === 'connecting') await api.cameraStop()
      else await api.cameraStart()
      await refresh()
    } finally {
      setBusy(false)
    }
  }

  const frameAge = status?.last_frame_at ? (now - Date.parse(status.last_frame_at)) / 1000 : null

  return (
    <Panel
      className={className}
      icon={Camera}
      title="Live camera"
      subtitle={status ? `USB camera · index ${status.device_index} · ${status.backend.toUpperCase()}` : 'Connecting to camera service…'}
      bodyClass="p-0"
      actions={
        <>
          <StateBadge state={apiError ? 'offline' : status?.state} />
          <Button size="sm" variant="ghost" icon={Maximize2} aria-label="Fullscreen"
            disabled={!showStream} onClick={() => frameRef.current?.requestFullscreen?.()} />
          <Button size="sm" icon={streaming || status?.state === 'connecting' ? Square : Play}
            onClick={toggle} disabled={busy || !!apiError}>
            {streaming || status?.state === 'connecting' ? 'Stop' : 'Start'}
          </Button>
        </>
      }
    >
      <div ref={frameRef} className={clsx('relative w-full overflow-hidden rounded-b-2xl bg-black', tall ? 'aspect-video min-h-[200px] sm:min-h-[260px]' : 'aspect-video min-h-[200px]')}>
        {showStream ? (
          <>
            <MjpegImage src={api.cameraStreamUrl(nonce)} onError={() => setImgFailed(true)} />
            <div className="pointer-events-none absolute inset-x-0 top-0 flex items-start justify-between p-3">
              <span className="inline-flex items-center gap-1.5 rounded-full bg-black/40 px-2.5 py-1 font-mono text-[11px] font-medium tracking-wider text-white backdrop-blur-md">
                <span className="size-2 animate-pulse-dot rounded-full bg-crit" aria-hidden /> LIVE
              </span>
              <span className="rounded-full bg-black/40 px-2.5 py-1 font-mono text-[11px] text-white/90 backdrop-blur-md tabular">
                {status?.width}×{status?.height} · {status?.fps ?? '—'} fps
              </span>
            </div>
            <div className="pointer-events-none absolute inset-x-0 bottom-0 flex justify-end p-3">
              <span className={clsx('rounded-full bg-black/40 px-2.5 py-1 font-mono text-[11px] backdrop-blur-md tabular',
                frameAge != null && frameAge > 2 ? 'text-warn-ink' : 'text-white/85')}>
                last frame {clock(status?.last_frame_at)}
              </span>
            </div>
          </>
        ) : (
          <Offline
            state={apiError ? 'offline' : status?.state ?? 'connecting'}
            message={apiError ?? status?.error ?? (imgFailed ? 'The stream connection dropped. Retrying when the camera reports frames again.' : null)}
            onRetry={() => { setImgFailed(false); setNonce(Date.now()); refresh() }}
          />
        )}
      </div>
    </Panel>
  )
}

/**
 * Chrome keeps an MJPEG connection open after its <img> is removed from the
 * DOM, and each leaked stream eats one of the ~6 connections per host, which
 * eventually stalls every API call. Clearing `src` on unmount aborts it.
 */
function MjpegImage({ src, onError }: { src: string; onError: () => void }) {
  const ref = useRef<HTMLImageElement>(null)
  useEffect(() => {
    const el = ref.current
    if (!el) return
    el.src = src
    return () => {
      el.src = 'data:,'
    }
  }, [src])
  return <img ref={ref} alt="Live USB camera feed" className="h-full w-full object-contain" onError={onError} />
}

function StateBadge({ state }: { state?: string }) {
  if (state === 'streaming') return <Badge tone="ok" icon={Camera}>Streaming</Badge>
  if (state === 'connecting') return <Badge tone="info" icon={Loader2} spin>Connecting</Badge>
  if (state === 'stopped') return <Badge tone="neutral" icon={CameraOff}>Stopped</Badge>
  if (state === 'offline') return <Badge tone="crit" icon={CameraOff}>API offline</Badge>
  return <Badge tone="crit" icon={CameraOff}>No signal</Badge>
}

function Offline({ state, message, onRetry }: { state: string; message: string | null; onRetry: () => void }) {
  const headline =
    state === 'stopped' ? 'Camera capture is stopped'
      : state === 'connecting' ? 'Opening camera…'
        : state === 'offline' ? 'Camera service unreachable'
          : 'No camera signal'
  return (
    <div className="grid-texture absolute inset-0 flex items-center justify-center bg-panel-2/80 p-4">
      <div className="glass-strong max-w-md rounded-2xl border border-glass-edge p-4 text-left">
        <div className="flex items-center gap-2">
          {state === 'connecting' ? <Loader2 className="size-4 animate-spin text-info-ink" /> : <CameraOff className="size-4 text-crit-ink" />}
          <p className="text-sm font-semibold">{headline}</p>
        </div>
        {message && <p className="mt-2 text-xs text-ink-2">{message}</p>}
        {state !== 'connecting' && state !== 'stopped' && (
          <ul className="mt-3 list-disc space-y-1 pl-4 text-xs text-ink-3">
            <li>Check the USB cable and that the camera LED is on.</li>
            <li>Close other apps using the camera (Teams, Zoom, OBS, browser tabs).</li>
            <li>Windows: Settings → Privacy &amp; security → Camera → allow desktop apps.</li>
            <li>Wrong device? Virtual cameras (e.g. OBS) can take an index. Change it in Settings or set <code className="font-mono text-ink-2">CAMERA_INDEX</code>.</li>
          </ul>
        )}
        <p className="mt-3 text-[11px] text-ink-3">No placeholder footage is shown in place of a missing camera.</p>
        {state !== 'connecting' && (
          <Button size="sm" className="mt-3" icon={RefreshCw} onClick={onRetry}>Retry</Button>
        )}
      </div>
    </div>
  )
}
