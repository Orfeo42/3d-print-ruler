"""Articulated print-in-place ruler for miniature wargames (zigzag pill-chain).

    Segment - Connector - Segment - Connector - ... - Segment

A Segment is a stadium-shaped bar (rounded both ends) with a rivet dropping
below each tip. A Connector is a smaller stadium plate sitting entirely
below the Segments (a different Z layer), with one round hole per end.

Two Segments never share a single pivot point - each has its own pivot into
the Connector below it, HOLE_SPACING apart. That is what makes the joint
collision-free through a wide fold range: a single shared pivot forces a
mating bar to be simultaneously "thin enough for the hole" and "wide enough
to be the bar" at the same radius, which is impossible (verified empirically
while prototyping this - see git history). Two separate pivots sidestep it
entirely.

Each rivet is a stepped pin: a plain shaft through the Connector's narrow
collar bore (PIN_CLEARANCE radial gap, spins free), then a HEAD wider than
the bore. The Connector itself is stepped to match: a narrow bore through
the top CONNECTOR_T collar, then a wider POCKET below it (radius = head
radius + POCKET_CLEARANCE) that the head spins inside with real clearance
on every side. The pocket is open at the Connector's bottom face, but the
head sits FLUSH_RECESS above that face - so the Connector's flat underside
(not the rivet head) is what a table/hand ever touches, keeping the whole
piece flat and giving the head a recess ("indent") shaped to fit it. The
head is printed already past the narrow bore (print-in-place, not
press-fit), so the two pieces are permanently captured in Z and cannot be
pulled apart, while still spinning freely in X/Y.

Each Segment's own tip cap is concentric with its own pivot (see BAR_W), so
it sweeps the same fixed disc regardless of fold angle. Sizing that disc to
HOLE_SPACING keeps neighbouring Segments TIP_GAP apart at every angle, not
just at rest - the top reads as one continuous surface with (by design) no
Connector visible except a hairline at each joint. The chain's two free
ends (first Segment's left tip, last Segment's right tip) are uncapped, so
NUM_LINKS Segments add up to almost exactly NUM_LINKS * SEG_LEN of total
length (plus the negligible TIP_GAP slack between them).

Measured on this geometry (see the collision-sweep check in the README):
each Segment independently swings about 83 degrees off straight with zero
collision, against both its Connector and its fixed neighbour, before two
neighbours start to pinch shut on each other directly - the same limit any
real hinge has. That is a real trade-off from the wider Segment needed to
close the top gap (down from ~140 degrees at the old, narrower width) - if
a tighter zigzag fold is needed later, HOLE_SPACING/BAR_W can be retuned.
"""

from pathlib import Path
from typing import Final

from build123d import (
    Align,
    Box,
    Compound,
    Cylinder,
    Part,
    Pos,
    Wire,
    export_step,
    export_stl,
)

SEG_LEN: Final[float] = 25.4  # Segment length (pivot centre to pivot centre) = 1"
BAR_T: Final[float] = 3.2  # Segment thickness (flat, single print job)

PIN_D: Final[float] = 3.4
PIN_CLEARANCE: Final[float] = 0.5  # radial gap for a free-spinning FDM print-in-place pivot; loosen if joints fuse

CONNECTOR_T: Final[float] = 2.2  # Connector collar thickness (solid part above the pocket)
Z_GAP: Final[float] = 0.15  # air gap between Segment underside and Connector top, so their caps never touch/fuse
HEAD_GAP: Final[float] = 0.3  # air gap above the rivet head (inside the pocket), so the head spins free
HEAD_T: Final[float] = 0.8  # rivet head thickness
HEAD_MARGIN: Final[float] = 0.6  # how much wider the rivet head is than the hole -> positive Z retention
POCKET_CLEARANCE: Final[float] = 0.4  # radial gap between the rivet head and the Connector's pocket wall
FLUSH_RECESS: Final[float] = 0.15  # how far the Connector's flat bottom sits below the rivet head -> head never bears load

HOLE_SPACING: Final[float] = 12.0  # distance between the Connector's two hole centres
CONN_W: Final[float] = 8.0  # Connector plate width

# Cap radius stays this much inside its own plate/bar half-width, so the
# round cap never overhangs a flat side edge (a fusion-fragile tangency).
_CAP_MARGIN: Final[float] = 0.3

TIP_GAP: Final[float] = 0.2  # gap between neighbouring Segment tip caps at rest - just above 0 so the top reads as one continuous surface without segments ever fusing in the slicer
# BAR_W is derived, not picked: each Segment's own tip cap must be concentric
# with its own pivot (it always is - see _segment) so the cap never moves
# with fold angle. Sizing cap_r to exactly HOLE_SPACING/2 - TIP_GAP/2 makes
# neighbouring Segments sit TIP_GAP apart at every fold angle, not just at
# rest - two fixed circles around two fixed points, always that far apart.
BAR_W: Final[float] = 2 * ((HOLE_SPACING - TIP_GAP) / 2 + _CAP_MARGIN)  # Segment width

NUM_LINKS: Final[int] = 6

# The two outer ends of the whole chain have no cap (see build_ruler), so the
# total length is exact Segment length plus the tiny rest-gap between them -
# no leftover cap overhang at the ends.
TOTAL_LEN: Final[float] = NUM_LINKS * SEG_LEN + (NUM_LINKS - 1) * TIP_GAP

type _Align3 = tuple[Align, Align, Align]
_XALIGN: Final[_Align3] = (Align.MIN, Align.CENTER, Align.MIN)
_ALIGN: Final[_Align3] = (Align.CENTER, Align.CENTER, Align.MIN)
_TOP_ALIGN: Final[_Align3] = (Align.CENTER, Align.CENTER, Align.MAX)

# build123d's boolean operators (+/-) are typed to return whatever solid
# kind results, including a Compound (expected here: our clearance gaps are
# deliberate, so unioning non-touching solids yields one) or, for
# degenerate/edge shapes, a Wire.
type PartLike = Part | Compound | Wire


def _rivet(hole_r: float) -> PartLike:
    """Segment-side pin: shaft through the hole, then a head too wide to pull back out."""
    pin_r: float = PIN_D / 2
    head_r: float = hole_r + HEAD_MARGIN
    shaft_bottom: float = -(Z_GAP + CONNECTOR_T + HEAD_GAP)
    head_bottom: float = shaft_bottom - HEAD_T

    shaft: PartLike = Cylinder(radius=pin_r, height=0.5 - shaft_bottom, align=_ALIGN)
    shaft = Pos(0, 0, shaft_bottom) * shaft
    head: PartLike = Pos(0, 0, head_bottom) * Cylinder(radius=head_r, height=HEAD_T, align=_ALIGN)
    return shaft + head


def _segment(*, left_joint: bool = True, right_joint: bool = True) -> PartLike:
    """Stadium-shaped bar with a rivet dropping below each jointed tip.

    A tip with no neighbour (the two free ends of the whole chain) gets no
    cap and no rivet - there is no Connector there to round off for or pivot
    into, so it stays a flat cut end instead of dead weight."""
    hole_r: float = (PIN_D + PIN_CLEARANCE) / 2
    cap_r: float = BAR_W / 2 - _CAP_MARGIN
    part: PartLike = Box(SEG_LEN, BAR_W, BAR_T, align=_XALIGN)
    for x, jointed in ((0.0, left_joint), (SEG_LEN, right_joint)):
        if not jointed:
            continue
        cap: PartLike = Pos(x, 0, 0) * Cylinder(radius=cap_r, height=BAR_T, align=_ALIGN)
        part = part + cap + Pos(x, 0, 0) * _rivet(hole_r)
    return part


def _connector() -> PartLike:
    """Stadium-shaped plate below the Segments. Each end is a stepped bore:
    a narrow collar bore (shaft clearance) on top, then a wide pocket
    (head clearance) that stays open at the Connector's own flat bottom
    face, with the rivet head recessed FLUSH_RECESS above that face."""
    hole_r: float = (PIN_D + PIN_CLEARANCE) / 2
    head_r: float = hole_r + HEAD_MARGIN
    pocket_r: float = head_r + POCKET_CLEARANCE
    cap_r: float = CONN_W / 2 - _CAP_MARGIN

    top: float = -Z_GAP
    collar_bottom: float = top - CONNECTOR_T
    head_bottom: float = collar_bottom - HEAD_GAP - HEAD_T
    bottom: float = head_bottom - FLUSH_RECESS
    thickness: float = top - bottom

    part: PartLike = Pos(0, 0, top) * Box(HOLE_SPACING, CONN_W, thickness, align=_TOP_ALIGN)
    for x in (-HOLE_SPACING / 2, HOLE_SPACING / 2):
        cap: PartLike = Pos(x, 0, top) * Cylinder(radius=cap_r, height=thickness, align=_TOP_ALIGN)
        part = part + cap
    for x in (-HOLE_SPACING / 2, HOLE_SPACING / 2):
        collar_bore: PartLike = Pos(x, 0, top) * Cylinder(radius=hole_r, height=top - collar_bottom, align=_TOP_ALIGN)
        pocket: PartLike = Pos(x, 0, collar_bottom) * Cylinder(radius=pocket_r, height=collar_bottom - bottom, align=_TOP_ALIGN)
        part = part - collar_bore - pocket
    return part


def build_ruler(num_links: int = NUM_LINKS) -> PartLike:
    """Straight chain of `num_links` Segments joined by Connectors below them.

    Only the two free ends of the whole chain (first Segment's left tip,
    last Segment's right tip) go uncapped/unjointed - every internal tip
    still gets its cap and rivet."""
    connector: PartLike = _connector()

    cursor: float = 0.0
    chain: PartLike | None = None
    for i in range(num_links):
        segment: PartLike = _segment(left_joint=i > 0, right_joint=i < num_links - 1)
        piece: PartLike = Pos(cursor, 0, 0) * segment
        chain = piece if chain is None else chain + piece
        cursor += SEG_LEN
        if i < num_links - 1:
            conn_center: float = cursor + HOLE_SPACING / 2
            chain = chain + Pos(conn_center, 0, 0) * connector
            cursor += HOLE_SPACING
    assert chain is not None
    return chain


def export_all(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    part: PartLike = build_ruler()
    stl_path: Path = out_dir / "ruler.stl"
    step_path: Path = out_dir / "ruler.step"
    export_stl(part, str(stl_path))
    export_step(part, str(step_path))
    print(f"wrote {stl_path}")
    print(f"wrote {step_path}")
    print(f"extended length: {TOTAL_LEN:.1f}mm ({NUM_LINKS} segments)")


def preview() -> None:
    """Show the ruler in the VSCode OCP CAD Viewer (open, port 3939)."""
    from ocp_vscode import show

    show(build_ruler())


if __name__ == "__main__":
    try:
        preview()
    except Exception as exc:  # viewer not running -> just export
        print(f"preview skipped ({type(exc).__name__}: {exc})")
    export_all(Path(__file__).resolve().parents[2] / "output")
