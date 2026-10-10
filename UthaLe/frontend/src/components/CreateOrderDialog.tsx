import clsx from 'clsx'
import { Minus, PackagePlus, Plus, X } from 'lucide-react'
import { useEffect, useMemo, useState } from 'react'
import { api } from '../lib/api'
import type { CatalogItem, Order, Priority } from '../types'
import { Button, ErrorNote, inputClass } from './ui'

export function CreateOrderDialog({ open, onClose, onCreated }: {
  open: boolean
  onClose: () => void
  onCreated: (o: Order) => void
}) {
  const [catalog, setCatalog] = useState<CatalogItem[]>([])
  const [destinations, setDestinations] = useState<string[]>(['BAY-1'])
  const [qty, setQty] = useState<Record<string, number>>({})
  const [destination, setDestination] = useState('BAY-1')
  const [priority, setPriority] = useState<Priority>('normal')
  const [assign, setAssign] = useState(true)
  const [note, setNote] = useState('')
  const [error, setError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)
  const [filter, setFilter] = useState('')

  useEffect(() => {
    if (!open) return
    api.catalog().then(setCatalog).catch((e) => setError(e.message))
    api.destinations().then(setDestinations).catch(() => {})
  }, [open])

  useEffect(() => {
    if (!open) return
    const onKey = (e: KeyboardEvent) => e.key === 'Escape' && onClose()
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [open, onClose])

  const shelves = useMemo(() => {
    const f = filter.trim().toLowerCase()
    const groups: Record<string, CatalogItem[]> = {}
    for (const c of catalog) {
      if (f && !c.name.toLowerCase().includes(f) && !c.sku.includes(f)) continue
      ;(groups[c.shelf] ??= []).push(c)
    }
    return Object.entries(groups)
  }, [catalog, filter])

  const lines = Object.entries(qty).filter(([, n]) => n > 0)

  if (!open) return null

  const bump = (sku: string, d: number) =>
    setQty((q) => ({ ...q, [sku]: Math.max(0, Math.min(10, (q[sku] ?? 0) + d)) }))

  async function submit() {
    setBusy(true)
    setError(null)
    try {
      const order = await api.createOrder({
        items: lines.map(([sku, quantity]) => ({ sku, quantity })),
        destination, priority, assign_robot: assign, note: note.trim() || undefined,
      })
      setQty({})
      setNote('')
      onCreated(order)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(false)
    }
  }

  return (
    <div className="fixed inset-0 z-50 flex items-end justify-center bg-black/30 backdrop-blur-md sm:items-center sm:p-6" onMouseDown={onClose}>
      <div role="dialog" aria-modal="true" aria-labelledby="new-order-title"
        className="flex max-h-[92vh] w-full max-w-3xl glass-strong animate-fade-in flex-col rounded-t-3xl border border-glass-edge sm:rounded-3xl"
        onMouseDown={(e) => e.stopPropagation()}>
        <header className="flex items-center gap-3 border-b border-line px-5 py-4">
          <PackagePlus className="size-5 text-brand" />
          <div className="flex-1">
            <h2 id="new-order-title" className="text-base font-semibold">New order</h2>
            <p className="text-xs text-ink-3">SKUs and shelf slots from the dark-store inventory</p>
          </div>
          <Button variant="ghost" size="sm" icon={X} onClick={onClose} aria-label="Close" />
        </header>

        <div className="grid min-h-0 flex-1 gap-5 overflow-y-auto p-5 md:grid-cols-[1fr_240px]">
          <div className="min-w-0">
            <input className={inputClass} placeholder="Filter products…" value={filter} onChange={(e) => setFilter(e.target.value)} />
            <div className="mt-3 space-y-4">
              {shelves.map(([shelf, items]) => (
                <div key={shelf}>
                  <p className="mb-1.5 text-[11px] font-medium uppercase tracking-wider text-ink-3">Shelf {shelf}</p>
                  <div className="grid gap-1.5 sm:grid-cols-2">
                    {items.map((c) => {
                      const n = qty[c.sku] ?? 0
                      return (
                        <div key={c.sku} className={clsx('flex items-center gap-2 rounded-xl border px-3 py-2 transition-colors',
                          n ? 'border-brand/40 bg-brand/10' : 'border-transparent bg-panel-2')}>
                          <button type="button" className="min-w-0 flex-1 text-left" onClick={() => bump(c.sku, n ? 0 : 1)}>
                            <p className="truncate text-[13px] text-ink">{c.name}</p>
                            <p className="font-mono text-[11px] text-ink-3">{c.location}</p>
                          </button>
                          {n > 0 ? (
                            <div className="flex items-center gap-1">
                              <button type="button" aria-label={`Fewer ${c.name}`} className="rounded p-1 text-ink-2 hover:bg-panel-3" onClick={() => bump(c.sku, -1)}><Minus className="size-3.5" /></button>
                              <span className="w-4 text-center text-sm tabular">{n}</span>
                              <button type="button" aria-label={`More ${c.name}`} className="rounded p-1 text-ink-2 hover:bg-panel-3" onClick={() => bump(c.sku, 1)}><Plus className="size-3.5" /></button>
                            </div>
                          ) : (
                            <button type="button" aria-label={`Add ${c.name}`} className="rounded p-1 text-ink-3 hover:bg-panel-3 hover:text-ink" onClick={() => bump(c.sku, 1)}><Plus className="size-4" /></button>
                          )}
                        </div>
                      )
                    })}
                  </div>
                </div>
              ))}
            </div>
          </div>

          <aside className="space-y-4">
            <label className="block">
              <span className="text-[11px] font-medium uppercase tracking-wider text-ink-3">Dispatch zone</span>
              <select className={clsx(inputClass, 'mt-1.5')} value={destination} onChange={(e) => setDestination(e.target.value)}>
                {destinations.map((d) => <option key={d}>{d}</option>)}
              </select>
            </label>
            <div>
              <span className="text-[11px] font-medium uppercase tracking-wider text-ink-3">Priority</span>
              <div className="mt-1.5 grid grid-cols-3 gap-0.5 rounded-full bg-panel-3 p-0.5">
                {(['low', 'normal', 'high'] as Priority[]).map((p) => (
                  <button key={p} type="button" onClick={() => setPriority(p)}
                    className={clsx('h-7 rounded-full text-xs capitalize transition-all', priority === p ? 'bg-elevated text-ink shadow-[0_1px_3px_rgba(0,0,0,0.12)]' : 'text-ink-2 hover:text-ink')}>{p}</button>
                ))}
              </div>
            </div>
            <label className="flex items-start gap-2 text-[13px] text-ink-2">
              <input type="checkbox" className="mt-0.5 accent-[var(--color-brand)]" checked={assign} onChange={(e) => setAssign(e.target.checked)} />
              <span>Assign to <span className="font-mono">SIM-STRETCH-01</span> now <span className="block text-[11px] text-ink-3">Simulated robot</span></span>
            </label>
            <label className="block">
              <span className="text-[11px] font-medium uppercase tracking-wider text-ink-3">Note</span>
              <input className={clsx(inputClass, 'mt-1.5')} maxLength={200} value={note} onChange={(e) => setNote(e.target.value)} placeholder="Optional" />
            </label>
            <div className="rounded-xl bg-panel-2 p-3 text-xs">
              <p className="font-medium text-ink-2">Summary</p>
              {lines.length === 0 ? <p className="mt-1 text-ink-3">No items selected</p> : (
                <ul className="mt-1.5 space-y-0.5 text-ink-2">
                  {lines.map(([sku, n]) => (
                    <li key={sku} className="flex justify-between gap-2">
                      <span className="truncate">{catalog.find((c) => c.sku === sku)?.name ?? sku}</span><span className="tabular">×{n}</span>
                    </li>
                  ))}
                </ul>
              )}
            </div>
          </aside>
        </div>

        <footer className="flex flex-wrap items-center justify-end gap-2 border-t border-line px-5 py-3">
          <div className="mr-auto min-w-0 flex-1"><ErrorNote error={error} /></div>
          <Button variant="ghost" onClick={onClose}>Cancel</Button>
          <Button variant="primary" icon={PackagePlus} disabled={busy || lines.length === 0} onClick={submit}>
            Create order
          </Button>
        </footer>
      </div>
    </div>
  )
}
