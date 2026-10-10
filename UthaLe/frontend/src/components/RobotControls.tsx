import clsx from 'clsx'
import { FlaskConical, OctagonX, Pause, Play, RotateCcw, Sliders, Zap } from 'lucide-react'
import { useState } from 'react'
import { api } from '../lib/api'
import { usePoll } from '../lib/hooks'
import type { Anomaly, RobotStatus } from '../types'
import { Badge, Button, Panel, inputClass } from './ui'

const anomalies: { value: Anomaly; label: string; expect: string }[] = [
  { value: 'weight_mismatch', label: 'Weight mismatch', expect: 'QC → REVIEW' },
  { value: 'nfc_missing', label: 'NFC tag not read', expect: 'QC → REVIEW' },
  { value: 'sku_mismatch', label: 'Wrong item placed', expect: 'QC → FAIL' },
  { value: 'grasp_failure', label: 'Grasp failure', expect: 'Pick fails ×2 → QC FAIL' },
]

/**
 * Mission controls. With the simulated adapter these only drive the simulator.
 * The stop button is a software stop. It is deliberately NOT presented as an
 * emergency stop: no physical robot is wired to it.
 */
export function RobotControls({ robot, onChange }: { robot: RobotStatus | null; onChange: () => void }) {
  const [msg, setMsg] = useState<{ ok: boolean; text: string } | null>(null)
  const [busy, setBusy] = useState(false)
  const [orderId, setOrderId] = useState('')
  const [anomaly, setAnomaly] = useState<Anomaly>('weight_mismatch')
  const { data: queued } = usePoll(() => api.orders('queued'), 3000)

  async function send(command: string, extra: Parameters<typeof api.robotCommand>[1] = {}) {
    setBusy(true)
    try {
      const r = await api.robotCommand(command, extra)
      setMsg({ ok: r.accepted, text: r.message })
      onChange()
    } catch (e) {
      setMsg({ ok: false, text: e instanceof Error ? e.message : String(e) })
    } finally {
      setBusy(false)
    }
  }

  const stopped = robot?.mode === 'stopped'
  const paused = robot?.mode === 'paused'
  const sim = robot?.simulated ?? true

  return (
    <Panel icon={Sliders} title="Mission control" actions={sim && <Badge tone="sim" icon={FlaskConical}>Simulated control</Badge>}>
      <div className="space-y-4">
        <div className="flex flex-col gap-2 sm:flex-row">
          <select className={inputClass} value={orderId} onChange={(e) => setOrderId(e.target.value)} aria-label="Order to start">
            <option value="">Next in queue (by priority)</option>
            {(queued ?? []).map((o) => (
              <option key={o.id} value={o.id}>{o.id} · {o.items.length} line(s) · {o.priority}{o.robot_id ? '' : ' · unassigned'}</option>
            ))}
          </select>
          <Button variant="primary" icon={Play} disabled={busy || stopped || !robot}
            onClick={() => send('start', orderId ? { order_id: orderId } : {})} className="shrink-0">
            Start mission
          </Button>
        </div>
        <div className="grid grid-cols-2 gap-2">
          <Button icon={Pause} disabled={busy || paused || stopped || !robot} onClick={() => send('pause')}>Pause</Button>
          <Button icon={Play} disabled={busy || !paused} onClick={() => send('resume')}>Resume</Button>
        </div>

        <div className={clsx('rounded-2xl border p-3', stopped ? 'border-crit/50 bg-crit/12' : 'border-crit/20 bg-crit/6')}>
          <div className="flex flex-wrap items-center gap-2">
            {stopped ? (
              <Button variant="secondary" icon={RotateCcw} disabled={busy} onClick={() => send('reset')} className="flex-1">
                Reset software stop
              </Button>
            ) : (
              <Button variant="danger" icon={OctagonX} disabled={busy || !robot} onClick={() => send('stop')} className="h-11 flex-1 text-sm font-semibold">
                Software stop{sim ? ' (simulated)' : ''}
              </Button>
            )}
          </div>
          <p className="mt-2 text-[11px] leading-relaxed text-crit-ink/90">
            <b>Not an emergency stop.</b> This halts the {sim ? 'simulated' : ''} mission in software only. No physical robot
            is connected to this button. For real hardware, always use the robot&apos;s physical E-stop.
          </p>
        </div>

        {msg && (
          <p className={clsx('rounded-xl border px-3 py-2 text-xs', msg.ok ? 'border-ok/30 bg-ok/10 text-ok-ink' : 'border-warn/40 bg-warn/10 text-warn-ink')}>
            {msg.text}
          </p>
        )}

        {sim && (
          <div className="border-t border-line pt-4">
            <p className="flex items-center gap-1.5 text-[11px] font-medium uppercase tracking-wider text-ink-3">
              <Zap className="size-3.5" /> Inject simulated anomaly
            </p>
            <p className="mt-1 text-xs text-ink-3">Applies to the next order line, to demo QC exception handling.</p>
            <div className="mt-2 flex flex-col gap-2 sm:flex-row">
              <select className={inputClass} value={anomaly} onChange={(e) => setAnomaly(e.target.value as Anomaly)} aria-label="Anomaly">
                {anomalies.map((a) => <option key={a.value} value={a.value}>{a.label} ({a.expect})</option>)}
              </select>
              <Button variant="warn" disabled={busy} onClick={() => send('inject_anomaly', { anomaly })} className="shrink-0">Arm</Button>
            </div>
            {robot?.pending_anomaly && (
              <p className="mt-2 text-xs text-warn-ink">Armed: {anomalies.find((a) => a.value === robot.pending_anomaly)?.label}</p>
            )}
          </div>
        )}
      </div>
    </Panel>
  )
}
