import type { ReactNode } from 'react'
import { setThemeChoice, useThemeChoice, type ThemeChoice } from '../lib/theme'

// Inline icons (Lucide sun / moon / monitor) so the site needs no icon package.
const svg = (children: ReactNode) => (
  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden>
    {children}
  </svg>
)

const OPTIONS: [ThemeChoice, string, ReactNode][] = [
  ['light', 'Light', svg(<><circle cx="12" cy="12" r="4" /><path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M6.34 17.66l-1.41 1.41M19.07 4.93l-1.41 1.41" /></>)],
  ['dark', 'Dark', svg(<path d="M12 3a6 6 0 0 0 9 9 9 9 0 1 1-9-9Z" />)],
  ['system', 'Match system', svg(<><rect x="2" y="3" width="20" height="14" rx="2" /><path d="M8 21h8M12 17v4" /></>)],
]

export default function ThemeToggle() {
  const choice = useThemeChoice()
  return (
    <div className="seg" role="radiogroup" aria-label="Theme">
      {OPTIONS.map(([value, label, icon]) => (
        <button key={value} type="button" role="radio" aria-checked={choice === value} aria-label={label} title={label}
          onClick={() => setThemeChoice(value)}>
          {icon}
        </button>
      ))}
    </div>
  )
}
