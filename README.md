# articulated-ruler

Print-in-place articulated ruler for miniature wargames, zigzag pill-chain
style.

```
Segment - Connector - Segment - Connector - ... - Segment   (NUM_LINKS segments)
```

Each **Segment** is a 1" (25.4mm) stadium-shaped bar (rounded both ends). It
does not pivot directly on its neighbour — a **Connector** sits *below* it
(a different Z layer entirely), with a stepped bore per end: a narrow collar
bore on top (shaft clearance), then a wider pocket below it (head clearance,
open at the Connector's own flat bottom face). Each Segment drops a rivet
pin into that bore: a plain shaft through the collar, then — inside the
pocket — a head wider than the collar. The head sits recessed just above
the Connector's bottom face (`FLUSH_RECESS`), never touching a table or
hand, so the Connector's own flat underside is what the ruler actually
rests on. The joint is permanently captured in Z (can't be pulled apart)
while still spinning free in X/Y.

Two Segments never share a single pivot point. That's deliberate: a shared
pivot forces the mating part to be simultaneously narrow enough to fit the
pivot hole and wide enough to be the bar, at the same radius — which is
geometrically impossible for any single-thickness hinge (confirmed by
building and testing that exact version — it collided even lying flat,
before any rotation). Two independent pivots, spaced `HOLE_SPACING` apart
and offset below the Segments, sidestep the problem entirely.

Each Segment's own tip cap is centred on its own pivot (see `BAR_W` below),
so the cap sweeps the exact same disc no matter what angle that Segment is
folded to. `BAR_W` is picked so that disc's radius comes out to exactly
`HOLE_SPACING / 2 - TIP_GAP / 2` — two fixed discs, `TIP_GAP` apart, always,
at every fold angle, not just at rest. That's what keeps the Segments
looking like one continuous top surface, with the Connector hidden
underneath except for a hairline at each joint.

Everything ships as **one STL** — pieces stay separate, watertight solids
(real clearance, no touching faces) so they spin free once snapped after
printing, no assembly.

## Generate

```sh
uv run python src/articulated_ruler/ruler.py   # -> output/ruler.stl / ruler.step
```

## Print-in-place check

Every export should split into `NUM_LINKS + (NUM_LINKS - 1)` disjoint,
watertight bodies (6 Segments + 5 Connectors by default):

```sh
uv run python -c "
import trimesh
m = trimesh.load('output/ruler.stl')
b = m.split(only_watertight=False)
print(len(b), 'bodies, all watertight:', all(x.is_watertight for x in b))
"
```

If that count drops, something fused — raise `Z_GAP` (Segment-to-Connector
clearance) and/or `PIN_CLEARANCE` and regenerate.

## Measured fold range

Each Segment independently swings about **83 degrees off straight** with
zero collision against both its Connector and its fixed neighbour (checked
by rotating a Segment step-by-step and measuring real mesh-intersection
volume at every step — collision starts appearing between 82° and 88°).
This is down from ~140° at the old, narrower `BAR_W` — closing the top gap
needed a wider Segment, which is bulkier through a fold. Retune
`HOLE_SPACING` (and `BAR_W`, which tracks it) if more range is needed later.

## Test plan

1. Print flat on the bed (single `BAR_T`-thick layer stack), no supports.
2. Snap each joint free by hand — apply a little force, same as any other
   print-in-place articulated ruler.
3. Confirm each Segment spins freely around its Connector, and that the
   rivet can't be pulled out (try it — it shouldn't budge in Z).
4. Fold the whole chain into a zigzag / winding path for a packing check.

## Unknowns to confirm — edit constants in `src/articulated_ruler/ruler.py`

- **Target length**: `NUM_LINKS` * `SEG_LEN` is the extended length
  (`TOTAL_LEN`, printed on export) — the top gap between Segments is
  `TIP_GAP` (near 0 by design), and the chain's two free ends are uncapped,
  so there's no leftover overhang at either end.
- **Joint fit**: `PIN_CLEARANCE` (shaft-to-hole radial gap) and `Z_GAP` /
  `HEAD_GAP` (vertical air gaps) depend on your printer's dimensional
  accuracy. Defaults target a 0.4mm-nozzle FDM printer; loosen any of them
  if joints print fused, tighten if they're too loose/rickety.
- **Retention**: `HEAD_MARGIN` sets how much wider the rivet head is than
  the hole. Bigger = harder to ever pull apart, but needs more clearance
  underneath to print cleanly.
- **Flush bottom**: `FLUSH_RECESS` sets how far below the rivet head the
  Connector's own bottom face sits. Keep it small but nonzero — it must
  stay positive so the Connector's flat rim, not the head, is what bears
  load; too big and the head rattles inside its pocket.

## Tuning knobs

- `HOLE_SPACING` — distance between the Connector's two pivots; also the
  Connector's own length. **`BAR_W` is derived from it** (plus `TIP_GAP` and
  `_CAP_MARGIN`), not picked independently — change `HOLE_SPACING` and
  `BAR_W` follows automatically to keep the top gap at `TIP_GAP`.
- `TIP_GAP` — resting gap between neighbouring Segment tip caps. Keep it
  small but nonzero (0 risks the two caps fusing in the slicer).
- `CONN_W` — Connector width. Must leave enough wall around the pocket
  radius (`hole radius + HEAD_MARGIN + POCKET_CLEARANCE`) — widen it if a
  joint prints with a blown-out side wall.
- `BAR_T` — Segment thickness. `CONNECTOR_T` — Connector collar thickness
  (the solid part above the pocket; the pocket itself adds more below it).
- `PIN_D` — rivet shaft diameter. `POCKET_CLEARANCE` — radial clearance
  between the rivet head and the Connector's pocket wall.
