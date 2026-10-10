import { Activity, Route } from 'lucide-react'
import { ActivityFeed } from '../components/ActivityFeed'
import { CameraFeed } from '../components/CameraFeed'
import { RobotControls } from '../components/RobotControls'
import { RobotStatusCard } from '../components/RobotStatusCard'
import { TaskList } from '../components/TaskList'
import { EmptyState, Panel, ProgressBar } from '../components/ui'
import { PageHeader } from '../layouts/AppLayout'
import { api } from '../lib/api'
import { usePoll } from '../lib/hooks'

const ROBOT_EVENTS = new Set([
  'mission_started', 'task_started', 'item_picked', 'item_delivered', 'robot_command', 'exception', 'order_picked',
])

export function LiveRobotPage() {
  const robot = usePoll(api.robotStatus, 1000)
  const orderId = robot.data?.current_order_id ?? null
  const mission = usePoll(() => (orderId ? api.order(orderId) : Promise.resolve(null)), 1000, [orderId])
  const activity = usePoll(() => api.activity(80), 1500)
  const events = activity.data?.filter((e) => ROBOT_EVENTS.has(e.type)).slice(0, 30) ?? null

  return (
    <>
      <PageHeader title="Live robot" description="Camera feed, mission state and controls for the picking robot" />
      <div className="grid grid-cols-1 gap-4 xl:grid-cols-12">
        <div className="min-w-0 space-y-4 xl:col-span-8">
          <CameraFeed tall />
          <Panel icon={Route} title={orderId ? `Current mission · ${orderId}` : 'Current mission'}
            subtitle={mission.data ? `${mission.data.items.length} line(s) → ${mission.data.destination}` : undefined}
            bodyClass="p-0">
            {mission.data ? (
              <>
                <div className="px-4 pt-3"><ProgressBar value={mission.data.progress} /></div>
                <TaskList tasks={mission.data.tasks} activeId={robot.data?.current_task_id} />
              </>
            ) : (
              <EmptyState icon={Route} title="No mission running">
                Assigned orders start automatically. Use <b>Start mission</b> to pick a specific one.
              </EmptyState>
            )}
          </Panel>
        </div>
        <div className="min-w-0 space-y-4 xl:col-span-4">
          <RobotStatusCard robot={robot.data} error={robot.error} />
          <RobotControls robot={robot.data} onChange={robot.refresh} />
          <Panel icon={Activity} title="Robot events" bodyClass="max-h-[380px] overflow-y-auto p-0">
            <ActivityFeed events={events} compact />
          </Panel>
        </div>
      </div>
    </>
  )
}
