import {
  Crosshair, Hand, Navigation, PackageCheck, ScanSearch, ShieldCheck, Truck, type LucideIcon,
} from 'lucide-react'
import type { TaskType } from '../types'

export const taskIcons: Record<TaskType, LucideIcon> = {
  navigate: Navigation,
  identify: ScanSearch,
  pick: Hand,
  transport: Truck,
  place: Crosshair,
  verify: ShieldCheck,
  dispatch: PackageCheck,
}
