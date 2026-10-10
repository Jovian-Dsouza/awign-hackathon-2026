import { BrowserRouter, Route, Routes } from 'react-router-dom'
import { AppLayout } from './layouts/AppLayout'
import { AnalyticsPage } from './pages/Analytics'
import { LiveRobotPage } from './pages/LiveRobot'
import { NotFoundPage } from './pages/NotFound'
import { OrderDetailPage } from './pages/OrderDetail'
import { OrdersPage } from './pages/Orders'
import { OverviewPage } from './pages/Overview'
import { QcPage } from './pages/Qc'
import { SettingsPage } from './pages/Settings'
import { TasksPage } from './pages/Tasks'

export default function App() {
  return (
    <BrowserRouter>
      <Routes>
        <Route element={<AppLayout />}>
          <Route index element={<OverviewPage />} />
          <Route path="robot" element={<LiveRobotPage />} />
          <Route path="orders" element={<OrdersPage />} />
          <Route path="orders/:orderId" element={<OrderDetailPage />} />
          <Route path="tasks" element={<TasksPage />} />
          <Route path="qc" element={<QcPage />} />
          <Route path="analytics" element={<AnalyticsPage />} />
          <Route path="settings" element={<SettingsPage />} />
          <Route path="*" element={<NotFoundPage />} />
        </Route>
      </Routes>
    </BrowserRouter>
  )
}
