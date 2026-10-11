// Blueprint-style side elevation of the robot reaching into a shelf. Line art only; proportions are
// illustrative. Shapes use pathLength=1 so CSS can draw each stroke in.

function Link({ x, y, len, angle, w = 24 }: { x: number; y: number; len: number; angle: number; w?: number }) {
  return (
    <g transform={`translate(${x} ${y}) rotate(${angle})`}>
      <rect className="ln" pathLength={1} x={-w / 2} y={-w / 2} width={len + w} height={w} rx={w / 2} />
      <line className="ln faint" pathLength={1} x1={0} y1={0} x2={len} y2={0} />
    </g>
  )
}

function Wheel({ cx }: { cx: number }) {
  return (
    <g transform={`translate(${cx} 392)`}>
      <circle className="ln" pathLength={1} r={26} />
      <g clipPath="url(#wheel-clip)">
        {[-24, -12, 0, 12, 24].map((o) => (
          <line key={o} className="ln faint" pathLength={1} x1={o - 20} y1={-26} x2={o + 20} y2={26} />
        ))}
      </g>
      <circle className="ln" pathLength={1} r={8} />
    </g>
  )
}

function Callout({ x, y, tx, ty, label, anchor = 'start' }: { x: number; y: number; tx: number; ty: number; label: string; anchor?: 'start' | 'end' }) {
  return (
    <g className="callout-svg">
      <circle cx={x} cy={y} r={3} />
      <polyline className="ln faint" pathLength={1} points={`${x},${y} ${tx},${ty} ${tx + (anchor === 'start' ? 14 : -14)},${ty}`} />
      <text x={tx + (anchor === 'start' ? 20 : -20)} y={ty + 4} textAnchor={anchor}>
        {label}
      </text>
    </g>
  )
}

export default function RobotSketch() {
  return (
    <svg className="sketch" viewBox="0 0 660 470" role="img" aria-label="Line drawing of the UthaLe robot reaching into a shelf with its gripper">
      <defs>
        <clipPath id="wheel-clip">
          <circle r={22} />
        </clipPath>
        <pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse">
          <path d="M20 0H0V20" fill="none" className="grid-line" />
        </pattern>
      </defs>
      <rect className="grid-bg" x="0" y="0" width="660" height="470" rx="20" fill="url(#grid)" />

      {/* floor */}
      <line className="ln" pathLength={1} x1={30} y1={420} x2={640} y2={420} />

      {/* shelf */}
      <g>
        <line className="ln" pathLength={1} x1={505} y1={90} x2={505} y2={420} />
        <line className="ln" pathLength={1} x1={635} y1={90} x2={635} y2={420} />
        {[150, 240, 330].map((y) => (
          <rect key={y} className="ln" pathLength={1} x={505} y={y} width={130} height={8} />
        ))}
        <rect className="ln faint" pathLength={1} x={560} y={110} width={40} height={40} rx={3} />
        <rect className="ln faint" pathLength={1} x={548} y={286} width={30} height={44} rx={4} />
        <rect className="ln faint" pathLength={1} x={588} y={300} width={36} height={30} rx={3} />
        <rect className="ln faint" pathLength={1} x={580} y={190} width={38} height={50} rx={3} />
      </g>

      {/* camera view cone */}
      <path className="cone" d="M462 196 L560 150 L560 252 Z" />

      {/* the item being picked */}
      <rect className="ln accent" pathLength={1} x={522} y={194} width={32} height={46} rx={6} />
      <line className="ln accent faint" pathLength={1} x1={522} y1={204} x2={554} y2={204} />

      {/* base */}
      <rect className="ln" pathLength={1} x={84} y={352} width={250} height={22} rx={6} />
      <rect className="ln" pathLength={1} x={84} y={318} width={250} height={8} rx={3} />
      {[100, 200, 318].map((x) => (
        <line key={x} className="ln faint" pathLength={1} x1={x} y1={326} x2={x} y2={352} />
      ))}
      <Wheel cx={128} />
      <Wheel cx={290} />

      {/* Raspberry Pi */}
      <rect className="ln" pathLength={1} x={104} y={296} width={64} height={22} rx={3} />
      {[114, 124, 134, 144, 154].map((x) => (
        <line key={x} className="ln faint" pathLength={1} x1={x} y1={296} x2={x} y2={290} />
      ))}

      {/* arm */}
      <rect className="ln" pathLength={1} x={244} y={290} width={44} height={28} rx={6} />
      <Link x={266} y={290} len={124} angle={-58} />
      <Link x={331} y={185} len={100} angle={10} w={22} />
      <circle className="ln" pathLength={1} cx={266} cy={290} r={9} />
      <circle className="ln" pathLength={1} cx={331} cy={185} r={9} />
      <circle className="ln" pathLength={1} cx={430} cy={203} r={8} />

      {/* gripper + wrist camera */}
      <rect className="ln" pathLength={1} x={436} y={200} width={50} height={28} rx={5} />
      <rect className="ln accent" pathLength={1} x={444} y={188} width={22} height={12} rx={3} />
      <g className="jaw jaw-top">
        <path className="ln accent" pathLength={1} d="M486 203 L536 197 L536 205 L486 211 Z" />
      </g>
      <g className="jaw jaw-bottom">
        <path className="ln accent" pathLength={1} d="M486 217 L536 223 L536 231 L486 225 Z" />
      </g>

      {/* callouts */}
      <Callout x={455} y={188} tx={420} ty={110} label="wrist camera" anchor="end" />
      <Callout x={300} y={238} tx={210} ty={150} label="6-axis SO-101 arm" anchor="end" />
      <Callout x={136} y={296} tx={112} ty={250} label="Raspberry Pi" />
      <Callout x={290} y={392} tx={360} ty={448} label="mecanum wheels" />
      <Callout x={538} y={240} tx={560} ty={448} label="item" />

      <text className="fig" x={30} y={40}>
        FIG. 00 — SIDE ELEVATION
      </text>
      <text className="fig" x={30} y={58}>
        UTHALE PICKING ROBOT · ILLUSTRATIVE
      </text>
    </svg>
  )
}
