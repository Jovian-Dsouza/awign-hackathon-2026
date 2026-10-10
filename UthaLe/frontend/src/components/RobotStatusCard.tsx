import { Bot, FlaskConical, Unplug } from 'lucide-react'
import { Link } from 'react-router-dom'
import { timeAgo } from '../lib/format'
import { useNow } from '../lib/hooks'
import type { RobotStatus } from '../types'
import { Badge, Field, ModeBadge, Panel, SimTag } from './ui'

export function RobotStatusCard({ robot, error, className }: { robot: RobotStatus | null; error: string | null; className?: string }) {
  const now = useNow(1000)
  return (
    <Panel
      className={className}
      icon={Bot}
      title={robot ? robot.robot_id : 'Robot'}
      subtitle={robot?.model}
      actions={robot && (
        <>
          {robot.simulated && <SimTag label="DEMO / SIM" />}
          <ModeBadge mode={robot.mode} />
        </>
      )}
    >
      {!robot ? (
        <p className="text-xs text-ink-3">{error ?? 'Loading robot status…'}</p>
      ) : (
        <div className="space-y-4">
          {robot.simulated && (
            <div className="flex gap-2 rounded-xl border border-sim/25 bg-sim/10 px-3 py-2 text-xs text-sim-ink">
              <FlaskConical className="mt-0.5 size-3.5 shrink-0" />
              <span>Simulated adapter. Mission state and zone come from the simulator, <b>not</b> robot telemetry. No physical robot is being commanded.</span>
            </div>
          )}
          {robot.stop_reason && (
            <div className="rounded-xl border border-crit/30 bg-crit/10 px-3 py-2 text-xs text-crit-ink">
              {robot.stop_reason}. Reset it on the Live Robot page.
            </div>
          )}
          <dl className="grid grid-cols-2 gap-x-4 gap-y-3">
            <Field label="Connection">
              {robot.connected
                ? <Badge tone={robot.simulated ? 'sim' : 'ok'}>{robot.simulated ? 'Sim adapter' : 'Connected'}</Badge>
                : <Badge tone="crit" icon={Unplug}>Disconnected</Badge>}
            </Field>
            <Field label="Mission">
              {robot.current_order_id
                ? <Link className="font-mono text-info-ink hover:underline" to={`/orders/${robot.current_order_id}`}>{robot.current_order_id}</Link>
                : <span className="text-ink-3">None</span>}
            </Field>
            <div className="col-span-2">
              <Field label="Current task">
                <span className={robot.current_task_title ? '' : 'text-ink-3'}>{robot.current_task_title ?? 'Waiting for work'}</span>
              </Field>
            </div>
            <Field label={<span className="inline-flex items-center gap-1">Zone {robot.simulated && <span className="text-sim-ink">(sim)</span>}</span>}>
              {robot.zone ?? '—'}
            </Field>
            <Field label="Battery">
              {robot.battery_pct != null ? `${robot.battery_pct}%` : <span className="text-ink-3">No telemetry</span>}
            </Field>
            <Field label="Queue">{robot.queue_length} order{robot.queue_length === 1 ? '' : 's'}</Field>
            <Field label="Last update"><span className="tabular">{timeAgo(robot.last_update, now)}</span></Field>
          </dl>
        </div>
      )}
    </Panel>
  )
}
