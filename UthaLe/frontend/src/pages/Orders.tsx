import { PackagePlus, Sparkles } from 'lucide-react'
import { useState } from 'react'
import { useNavigate } from 'react-router-dom'
import { CreateOrderDialog } from '../components/CreateOrderDialog'
import { OrdersTable } from '../components/OrdersTable'
import { Button, ErrorNote, Panel, Tabs } from '../components/ui'
import { PageHeader } from '../layouts/AppLayout'
import { api } from '../lib/api'
import { usePoll } from '../lib/hooks'

type Filter = 'all' | 'active' | 'dispatched' | 'exception'

export function OrdersPage() {
  const { data, error, refresh } = usePoll(() => api.orders(), 2000)
  const [filter, setFilter] = useState<Filter>('all')
  const [dialog, setDialog] = useState(false)
  const [busy, setBusy] = useState(false)
  const navigate = useNavigate()

  const counts = {
    all: data?.length,
    active: data?.filter((o) => !['dispatched', 'exception'].includes(o.status)).length,
    dispatched: data?.filter((o) => o.status === 'dispatched').length,
    exception: data?.filter((o) => o.status === 'exception').length,
  }
  const rows = data?.filter((o) =>
    filter === 'all' ? true
      : filter === 'active' ? !['dispatched', 'exception'].includes(o.status)
        : o.status === filter) ?? null

  async function demo() {
    setBusy(true)
    try {
      await api.createDemoOrder()
      refresh()
    } finally {
      setBusy(false)
    }
  }

  return (
    <>
      <PageHeader
        title="Orders"
        description="Customer orders and their fulfilment status. Records persist in the backend database."
        actions={<>
          <Button icon={Sparkles} onClick={demo} disabled={busy}>Demo order</Button>
          <Button variant="primary" icon={PackagePlus} onClick={() => setDialog(true)}>New order</Button>
        </>}
      />
      {error && <div className="mb-4"><ErrorNote error={error} /></div>}
      <div className="mb-3">
        <Tabs value={filter} onChange={setFilter} options={[
          { value: 'all', label: 'All', count: counts.all },
          { value: 'active', label: 'Active', count: counts.active },
          { value: 'dispatched', label: 'Dispatched', count: counts.dispatched },
          { value: 'exception', label: 'Exceptions', count: counts.exception },
        ]} />
      </div>
      <Panel bodyClass="p-0">
        <OrdersTable orders={rows} empty="Create an order, or generate a demo order from the inventory." />
      </Panel>
      <CreateOrderDialog open={dialog} onClose={() => setDialog(false)}
        onCreated={(o) => { setDialog(false); navigate(`/orders/${o.id}`) }} />
    </>
  )
}
