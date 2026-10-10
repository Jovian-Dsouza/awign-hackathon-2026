import { useSyncExternalStore } from 'react'

export type ThemeChoice = 'system' | 'light' | 'dark'

const KEY = 'uthale.theme'
const EVENT = 'uthale-theme'
const media = () => window.matchMedia('(prefers-color-scheme: dark)')

export function getThemeChoice(): ThemeChoice {
  try {
    const v = localStorage.getItem(KEY)
    return v === 'light' || v === 'dark' ? v : 'system'
  } catch {
    return 'system'
  }
}

/** Stamp data-theme on <html> ("system" leaves it off so the OS preference applies). */
export function setThemeChoice(choice: ThemeChoice) {
  try {
    if (choice === 'system') localStorage.removeItem(KEY)
    else localStorage.setItem(KEY, choice)
  } catch {
    /* storage unavailable: still apply for this session */
  }
  if (choice === 'system') delete document.documentElement.dataset.theme
  else document.documentElement.dataset.theme = choice
  window.dispatchEvent(new Event(EVENT))
}

function subscribe(cb: () => void) {
  const mq = media()
  mq.addEventListener('change', cb)
  window.addEventListener(EVENT, cb)
  return () => {
    mq.removeEventListener('change', cb)
    window.removeEventListener(EVENT, cb)
  }
}

export function useThemeChoice(): ThemeChoice {
  return useSyncExternalStore(subscribe, getThemeChoice, () => 'system')
}

/** The theme actually on screen; charts need concrete colors for SVG attributes. */
export function useResolvedTheme(): 'light' | 'dark' {
  return useSyncExternalStore(
    subscribe,
    () => {
      const choice = getThemeChoice()
      return choice === 'system' ? (media().matches ? 'dark' : 'light') : choice
    },
    () => 'light',
  )
}
