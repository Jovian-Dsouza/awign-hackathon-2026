import { Compass } from 'lucide-react'
import { Link } from 'react-router-dom'
import { EmptyState } from '../components/ui'

export function NotFoundPage() {
  return (
    <EmptyState icon={Compass} title="Page not found">
      <Link to="/" className="text-info-ink hover:underline">Back to overview</Link>
    </EmptyState>
  )
}
