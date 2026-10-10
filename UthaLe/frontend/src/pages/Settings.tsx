import { Bot, Camera, Database, FlaskConical, Radio } from 'lucide-react'
import { useState } from 'react'
import { Button, ErrorNote, Panel, inputClass } from '../components/ui'
import { PageHeader } from '../layouts/AppLayout'
import { API_BASE, api } from '../lib/api'
import { usePoll } from '../lib/hooks'

function KV({ data }: { data?: Record<string, string | number | boolean | null> }) {
  if (!data) return <p className="text-xs text-ink-3">Loading…</p>
  return (
    <dl className="divide-y divide-line text-[13px]">
      {Object.entries(data).map(([k, v]) => (
        <div key={k} className="flex items-start justify-between gap-4 py-2">
          <dt className="text-ink-3">{k.replace(/_/g, ' ')}</dt>
          <dd className="min-w-0 break-all text-right font-mono text-xs text-ink">{String(v)}</dd>
        </div>
      ))}
    </dl>
  )
}

export function SettingsPage() {
  const settings = usePoll(api.settings, 5000)
  const cam = usePoll(api.cameraStatus, 2000)
  const [draft, setDraft] = useState<string | null>(null)
  const [msg, setMsg] = useState<string | null>(null)
  const [error, setError] = useState<string | null>(null)

  const index = draft ?? (cam.data ? String(cam.data.device_index) : '')

  async function apply() {
    setError(null)
    setMsg(null)
    try {
      const s = await api.cameraSetIndex(Number(index))
      setDraft(null)
      setMsg(`Camera switched to index ${s.device_index}. Watch the status below. Use the Overview feed to confirm it's the right device.`)
      cam.refresh()
      settings.refresh()
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    }
  }

  return (
    <>
      <PageHeader title="Settings" description="Camera, robot connection and system configuration" />
      <div className="grid gap-4 xl:grid-cols-2">
        <Panel icon={Camera} title="Camera">
          <div className="space-y-4">
            <div className="flex flex-col gap-2 sm:flex-row sm:items-end">
              <label className="block flex-1">
                <span className="text-[11px] font-medium uppercase tracking-wider text-ink-3">Device index</span>
                <input type="number" min={0} max={16} className={`${inputClass} mt-1.5`} value={index} onChange={(e) => setDraft(e.target.value)} />
              </label>
              <Button variant="primary" onClick={apply} disabled={index === ''}>Apply</Button>
              <Button onClick={() => api.cameraStart().then(cam.refresh)}>Start</Button>
              <Button onClick={() => api.cameraStop().then(cam.refresh)}>Stop &amp; release</Button>
            </div>
            {msg && <p className="text-xs text-ok-ink">{msg}</p>}
            <ErrorNote error={error} />
            <p className="text-xs text-ink-3">
              Runtime change only. To persist it, set <code className="font-mono text-ink-2">CAMERA_INDEX</code> in <code className="font-mono text-ink-2">backend/.env</code>.
              Indices shift when devices are plugged in or out, and virtual cameras (OBS, Snap Camera) take indices too.
            </p>
            <div className="rounded-xl bg-panel-2 px-3"><KV data={cam.data ? { ...cam.data } as Record<string, string | number | null> : undefined} /></div>
            <div className="rounded-xl bg-panel-2 px-3"><KV data={settings.data?.camera} /></div>
          </div>
        </Panel>

        <div className="space-y-4">
          <Panel icon={Bot} title="Robot connection">
            <div className="mb-3 flex gap-2 rounded-xl border border-sim/25 bg-sim/10 px-3 py-2 text-xs text-sim-ink">
              <FlaskConical className="mt-0.5 size-3.5 shrink-0" />
              <span>Only the simulated adapter is implemented. A real adapter (Stretch SDK / ROS 2) plugs in behind the <code className="font-mono">RobotAdapter</code> interface in <code className="font-mono">backend/app/services/robot.py</code>.</span>
            </div>
            <KV data={settings.data?.robot} />
          </Panel>
          <Panel icon={Radio} title="Sensors"><KV data={settings.data?.sensors} /></Panel>
          <Panel icon={Database} title="Storage & API">
            <KV data={settings.data ? { ...settings.data.storage, api_base: API_BASE || `${window.location.origin} (proxied to backend)` } : undefined} />
          </Panel>
        </div>
      </div>
    </>
  )
}
