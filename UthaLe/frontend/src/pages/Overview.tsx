import { Activity, ClipboardList, FlaskConical, PackagePlus, Sparkles } from 'lucide-react'
import { useState } from 'react'
import { Link, useNavigate } from 'react-router-dom'
import { ActivityFeed } from '../components/ActivityFeed'
import { CameraFeed } from '../components/CameraFeed'
import { CreateOrderDialog } from '../components/CreateOrderDialog'
import { KpiStrip } from '../components/KpiStrip'
import { OrdersTable } from '../components/OrdersTable'
import { RobotStatusCard } from '../components/RobotStatusCard'
import { Button, ErrorNote, Panel } from '../components/ui'
import { PageHeader } from '../layouts/AppLayout'
import { api } from '../lib/api'
import { usePoll } from '../lib/hooks'

export function OverviewPage() {
  const overview = usePoll(api.overview, 2000)
  const robot = usePoll(api.robotStatus, 1000)
  const activity = usePoll(() => api.activity(25), 1500)
  const orders = usePoll(() => api.orders('active'), 2000)
  const [dialog, setDialog] = useState(false)
  const [busy, setBusy] = useState(false)
  const navigate = useNavigate()

  async function demo() {
    setBusy(true)
    try {
      await api.createDemoOrder()
      orders.refresh()
      overview.refresh()
      activity.refresh()
    } finally {
      setBusy(false)
    }
  }

  return (
    <>
      <PageHeader
        title="Operations overview"
        description={
          <span className="inline-flex flex-wrap items-center gap-x-2 gap-y-1">
            Dark store · live camera, robot missions and order flow
            <span className="inline-flex items-center gap-1 rounded-full border border-sim/30 bg-sim/10 px-2 py-0.5 text-[11px] text-sim-ink">
              <FlaskConical className="size-3" /> Robot &amp; sensors in demo mode
            </span>
          </span>
        }
        actions={
          <>
            <Button icon={Sparkles} onClick={demo} disabled={busy}>Demo order</Button>
            <Button variant="primary" icon={PackagePlus} onClick={() => setDialog(true)}>New order</Button>
          </>
        }
      />
      {overview.error && <div className="mb-4"><ErrorNote error={overview.error} /></div>}

      <KpiStrip data={overview.data} />

      <div className="mt-4 grid grid-cols-1 gap-4 xl:grid-cols-12">
        <CameraFeed className="xl:col-span-8" />
        <RobotStatusCard className="xl:col-span-4" robot={robot.data} error={robot.error} />
      </div>

      <div className="mt-4 grid grid-cols-1 gap-4 xl:grid-cols-12">
        <Panel className="xl:col-span-8" icon={ClipboardList} title="Active orders" bodyClass="p-0"
          actions={<Link to="/orders" className="text-xs text-ink-3 hover:text-ink">View all →</Link>}>
          <OrdersTable orders={orders.data} empty={<>
            No active orders. <button type="button" className="text-info-ink hover:underline" onClick={() => setDialog(true)}>Create one</button> to start a picking mission.
          </>} />
        </Panel>
        <Panel className="xl:col-span-4" icon={Activity} title="Recent activity" bodyClass="max-h-[420px] overflow-y-auto p-0">
          <ActivityFeed events={activity.data} compact />
        </Panel>
      </div>

      <CreateOrderDialog open={dialog} onClose={() => setDialog(false)}
        onCreated={(o) => { setDialog(false); navigate(`/orders/${o.id}`) }} />
    </>
  )
}
