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

Each Segment's own tip is rounded to a cylinder of radius CAP_R centred on
its pivot, and that pivot sits inset CAP_R from the Segment's own physical
edge (see CAP_R) - not at the edge - so the round tip lands exactly flush
with it, no overhang. Carving the tip down to that cylinder matters, not
just adding a same-radius disc on top of the still-square corner - a square
corner reaches sqrt(2)*CAP_R from the pivot, further than the round tip, so
it would hit the Connector within the first few degrees of fold (an earlier
version of this file got exactly that wrong - see git history). Every
Segment's own footprint stays exactly its own SegmentSpec.length, whether
jointed or a free end (which gets a corner fillet instead, same radius,
also flush). Neighbouring Segments end up exactly TIP_GAP apart at every
joint, at every fold angle - the Connector's own HOLE_SPACING footprint is
carved out of the two neighbouring Segments' own length budget, not added
on top of it (an earlier bug - see git history - made the ruler ~40% longer
than intended).

Each Segment's length/width/thickness is a SegmentSpec, not a fixed
constant - build_ruler() takes a list of them (default: NUM_LINKS copies of
SegmentSpec(), the plain 1"/25.4mm bar) and lays them out end to end with a
running cursor, so Segments of different lengths still line up. width must
stay >= 2*CAP_R + a margin (CAP_R is fixed globally, not per-Segment) or a
jointed tip's round cap has nowhere to fit.

Away from both tips, a Segment's plain middle thickens down to
CONNECTOR_BOTTOM_Z (see MID_MARGIN) - the same depth a joint already has
from the Connector stacked below it - so the ruler reads as one uniform
thickness end to end, not thin straps between thick beads. MID_MARGIN keeps
that thickened middle clear of the Connector's own footprint at every fold
angle (same fixed-disc argument as the tip caps).

Measured on this geometry (see the collision-sweep check in the README):
each Segment independently swings about 83 degrees off straight with zero
collision, against both its Connector and its fixed neighbour, before two
neighbours start to pinch shut on each other directly - the same limit any
real hinge has. That is a real trade-off from the wider Segment needed to
close the top gap (down from ~140 degrees at the old, narrower width) - if
a tighter zigzag fold is needed later, HOLE_SPACING/BAR_W can be retuned.
"""

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from build123d import (
    Align,
    Axis,
    Box,
    chamfer,
    Compound,
    Curve,
    Cylinder,
    Edge,
    Part,
    Pos,
    Wire,
    export_step,
    export_stl,
    fillet,
)

SEG_LEN: Final[float] = 25.4  # Segment length (pivot centre to pivot centre) = 1"
BAR_T: Final[float] = 3.2  # Segment thickness (flat, single print job)

PIN_D: Final[float] = 3.4
PIN_CLEARANCE: Final[float] = (
    0.5  # radial gap for a free-spinning FDM print-in-place pivot; loosen if joints fuse
)

CONNECTOR_T: Final[float] = (
    2.2  # Connector collar thickness (solid part above the pocket)
)
Z_GAP: Final[float] = (
    0.15  # air gap between Segment underside and Connector top, so their caps never touch/fuse
)
HEAD_GAP: Final[float] = (
    0.3  # air gap above the rivet head (inside the pocket), so the head spins free
)
HEAD_T: Final[float] = 0.8  # rivet head thickness
HEAD_MARGIN: Final[float] = (
    0.6  # how much wider the rivet head is than the hole -> positive Z retention
)
POCKET_CLEARANCE: Final[float] = (
    0.4  # radial gap between the rivet head and the Connector's pocket wall
)
FLUSH_RECESS: Final[float] = (
    0.15  # how far the Connector's flat bottom sits below the rivet head -> head never bears load
)

HOLE_SPACING: Final[float] = 12.0  # distance between the Connector's two hole centres
CONN_W: Final[float] = 8.0  # Connector plate width

# Full stack depth at a joint: Segment top (z=0) down to the Connector's own
# flat bottom. The Segment's plain middle (away from both tips) thickens
# down to this same depth, so the ruler reads as one uniform thickness
# instead of thin straps between thick joints.
CONNECTOR_BOTTOM_Z: Final[float] = -(
    Z_GAP + CONNECTOR_T + HEAD_GAP + HEAD_T + FLUSH_RECESS
)

# Keep-clear zone from each tip before the Segment's middle thickens - must
# clear the Connector's own end-cap reach (CONN_W / 2, its own cap is a true
# full-width semicircle - see _connector's cap_r) with a small buffer, or the
# thickened middle would collide with the Connector.
MID_MARGIN: Final[float] = CONN_W / 2 + 0.3

TIP_GAP: Final[float] = (
    0.2  # gap between neighbouring Segment tip caps at rest - just above 0 so the top reads as one continuous surface without segments ever fusing in the slicer
)

# CAP_R is the tip cap's radius, AND (see _segment) how far each jointed
# pivot sits inset from its own Segment's physical tip - the cap is always
# concentric with its own pivot, so sizing the inset to exactly CAP_R makes
# the cap land exactly on the Segment's own edge: no overhang, no exposed
# flat sliver, and neighbouring Segments end up exactly TIP_GAP apart at
# every fold angle (two fixed discs, CAP_R + CAP_R + TIP_GAP = HOLE_SPACING
# apart, always - see build_ruler's JOINT_PITCH). Critically, the Connector's
# HOLE_SPACING footprint is now carved OUT OF each Segment's own SEG_LEN
# budget instead of adding extra length beyond it.
CAP_R: Final[float] = (HOLE_SPACING - TIP_GAP) / 2
# Exactly 2*CAP_R (no extra margin): the tip's round cap is meant to be a
# true full-width semicircle, tangent to both flat side edges at the tip,
# not a smaller circle sitting inset with a flat lip either side.
BAR_W: Final[float] = 2 * CAP_R  # Segment width

# A free tip's round cap is built as a fillet() on the box's own corner
# edges, radius CAP_R. With width == 2*CAP_R exactly, that radius is exactly
# half the box's own width - a mathematically exact tangency the OCCT
# kernel refuses outright ("Failed creating a fillet ... try a smaller
# value"), not a size/quality issue. _TANGENT_EPS shaves a numerically-safe
# sliver off just the fillet radius (imperceptible well below FDM
# resolution) so the two corner fillets still meet almost exactly at the
# tip's centreline without asking the kernel for an exact tangency.
_TANGENT_EPS: Final[float] = 0.02
_FREE_TIP_FILLET_R: Final[float] = CAP_R - _TANGENT_EPS

# Small chamfer run around every Segment/Connector's top-face outer rim -
# softens the printed top edge (less prone to catching/chipping when the
# print-in-place joints are snapped free by hand) without touching the
# tip/joint geometry underneath, which stays exact-radius for collision
# clearance.
EDGE_CHAMFER: Final[float] = 0.4

# Same radius as EDGE_CHAMFER, used for the Segment's bottom rim - a fillet
# (smooth round) rather than a flat chamfer, but the same size so top and
# bottom read as the same bevel scale.
BOTTOM_FILLET_R: Final[float] = EDGE_CHAMFER

# At a JOINTED end, the thickened-middle slab (see CONNECTOR_BOTTOM_Z) still
# stops MID_MARGIN short of the tip (clearing the Connector's own
# footprint), so its own near-tip wall still meets the tip's flat underside
# in a sharp step - hidden under the Connector, but still a real sharp
# corner in the actual mesh. STEP_ROUND_R rounds that step too, same idea as
# rounding a free end's tip corner, just a different edge (the step's
# bottom edge - where the wall meets the thick slab's own flat underside -
# not a vertical tip corner). Clamped per-use to the slab's own height/run
# so it never asks the kernel for a radius bigger than the geometry it's
# rounding.
STEP_ROUND_R: Final[float] = 1.2

NUM_LINKS: Final[int] = 6

type _Align3 = tuple[Align, Align, Align]
_XALIGN: Final[_Align3] = (Align.MIN, Align.CENTER, Align.MIN)
_ALIGN: Final[_Align3] = (Align.CENTER, Align.CENTER, Align.MIN)
_TOP_ALIGN: Final[_Align3] = (Align.CENTER, Align.CENTER, Align.MAX)

# build123d's boolean operators (+/-) are typed to return whatever solid
# kind results, including a Compound (expected here: our clearance gaps are
# deliberate, so unioning non-touching solids yields one) or, for
# degenerate/edge shapes, a Wire.
type PartLike = Part | Compound | Wire


def _solid(shape: Part | Compound | Wire | Edge | Curve) -> PartLike:
    """Narrow a boolean-op result back to PartLike. build123d's +/- operator
    stubs are typed for the general case (any Shape, down to a bare Edge or
    Curve), but every operand here is already a solid Box/Cylinder/fillet
    result, so the result is always a Part, Compound, or Wire in practice -
    this asserts that instead of just casting past the checker."""
    if isinstance(shape, (Part, Compound, Wire)):
        return shape
    raise TypeError(f"expected a solid Part/Compound/Wire, got {type(shape).__name__}")


def _rim_edges(part: PartLike, *, top: bool) -> list[Edge]:
    """Outer-silhouette edges of a solid's top face (top=True) or bottom
    face (top=False) - the two Z extremes only, NOT every horizontal face.
    `filter_by(Axis.Z)` would also pick up the small internal shelf face
    inside each Connector's stepped bore (where the bore steps from the
    narrow collar to the wide pocket); that's horizontal too, but its
    "outer" edge is actually the pocket's bore rim, which self-intersects
    the vertical bore wall right next to it and makes fillet/chamfer reject
    the whole op. `outer_wire()` on the correct single face avoids that, and
    also leaves genuine inner hole loops (e.g. the collar bores) untouched."""
    faces_by_z = part.faces().sort_by(Axis.Z)
    face = faces_by_z[-1] if top else faces_by_z[0]
    return face.outer_wire().edges()


def _chamfer_horizontal_rims(part: PartLike, radius: float) -> PartLike:
    """Chamfer both the top and bottom outer rim - used for the Connector,
    which (unlike a Segment) has no reason to treat top/bottom differently."""
    rim_edges: list[Edge] = _rim_edges(part, top=True) + _rim_edges(part, top=False)
    return _solid(chamfer(rim_edges, radius))


@dataclass(frozen=True, slots=True)
class SegmentSpec:
    """One Segment's own length/width/thickness - free to differ per Segment
    (e.g. a wider handle segment, or a shorter end piece). `width` must stay
    >= 2 * CAP_R: CAP_R (the tip round radius) is fixed globally by
    HOLE_SPACING/TIP_GAP so every jointed tip still lines up with its
    Connector and its neighbour, regardless of that Segment's own width. The
    default (BAR_W) is exactly 2 * CAP_R, so the tip's round cap is a true
    full-width semicircle, not a smaller circle inset from the edge."""

    length: float = SEG_LEN
    width: float = BAR_W
    thickness: float = BAR_T


def _default_specs(num_links: int = NUM_LINKS) -> list[SegmentSpec]:
    return [SegmentSpec() for _ in range(num_links)]


def _total_length(specs: Sequence[SegmentSpec]) -> float:
    """Real extended length: sum of each Segment's own length, plus the
    negligible TIP_GAP slack at each of the joints between them."""
    return sum(spec.length for spec in specs) + (len(specs) - 1) * TIP_GAP


def _jointed_tip_bottom_rim_edges(
    part: PartLike, spec: SegmentSpec, *, left_joint: bool, right_joint: bool
) -> list[Edge]:
    """A jointed tip's own round nose, at the Segment's own bottom plane
    (z=0, NOT the deeper thickened-middle slab) - the face two neighbouring
    Segments face each other across TIP_GAP, its own rim never touched by
    any fillet/chamfer so far. This flat z=0 face is NOT its own separate
    face bounded to just the tip - it's fused, with no dividing edge at all,
    to the plain (not-yet-thickened) box area right behind the tip, so its
    bounding box runs way past the tip and a bbox-filtered *face* search
    misses it entirely. Instead, take that face's outer_wire and keep only
    the EDGES whose own bounding box sits fully within the tip's own CAP_R
    reach from the physical end - i.e. just the round nose arc, not the
    straight sides behind it."""
    edges: list[Edge] = []
    for jointed, in_tip_zone in (
        (left_joint, lambda bb: bb.max.X < CAP_R + 1e-6),
        (right_joint, lambda bb: bb.min.X > spec.length - CAP_R - 1e-6),
    ):
        if not jointed:
            continue
        for face in part.faces():
            bbox = face.bounding_box()
            if bbox.max.Z - bbox.min.Z > 1e-6 or abs(bbox.min.Z) > 1e-6:
                continue
            edges += [
                e for e in face.outer_wire().edges() if in_tip_zone(e.bounding_box())
            ]
    return edges


def _rivet(hole_r: float) -> PartLike:
    """Segment-side pin: shaft through the hole, then a head too wide to pull back out."""
    pin_r: float = PIN_D / 2
    head_r: float = hole_r + HEAD_MARGIN
    shaft_bottom: float = -(Z_GAP + CONNECTOR_T + HEAD_GAP)
    head_bottom: float = shaft_bottom - HEAD_T

    shaft: PartLike = Cylinder(radius=pin_r, height=0.5 - shaft_bottom, align=_ALIGN)
    shaft = Pos(0, 0, shaft_bottom) * shaft
    head: PartLike = Pos(0, 0, head_bottom) * Cylinder(
        radius=head_r, height=HEAD_T, align=_ALIGN
    )
    return shaft + head


def _segment(
    spec: SegmentSpec, *, left_joint: bool = True, right_joint: bool = True
) -> PartLike:
    """Stadium-shaped bar, always exactly `spec.length` long - no exceptions
    for the chain's free ends.

    A jointed tip's rivet sits inset CAP_R from that tip (not at it), and the
    tip region itself is carved down to a cylinder of radius CAP_R centred
    on that same pivot - genuinely round, not a disc sitting redundantly
    inside an already-square corner (a square corner reaches sqrt(2)*CAP_R
    from the pivot, further than the round cap, so it would hit the
    Connector on the very first few degrees of fold). A free tip (no
    neighbour, no Connector to pivot into) is rounded with a corner fillet
    instead, same radius, also flush with the edge. Either way the
    Segment's own box footprint never grows past `spec.length`.

    Away from both tips, the plain middle thickens down to
    CONNECTOR_BOTTOM_Z - the same depth a joint already has from the
    Connector stacked below it - so the whole Segment reads as one uniform
    thickness, not a thin strap between thick beads."""
    hole_r: float = (PIN_D + PIN_CLEARANCE) / 2
    box: Part = Box(spec.length, spec.width, spec.thickness, align=_XALIGN)

    free_edges: list[Edge] = []
    if not left_joint:
        free_edges += [
            e
            for e in box.edges().filter_by(Axis.Z)
            if e.bounding_box().min.X < spec.length / 2
        ]
    if not right_joint:
        free_edges += [
            e
            for e in box.edges().filter_by(Axis.Z)
            if e.bounding_box().min.X > spec.length / 2
        ]
    part: PartLike = (
        fillet(free_edges, radius=_FREE_TIP_FILLET_R) if free_edges else box
    )

    for pivot_x, region_lo, jointed in (
        (CAP_R, 0.0, left_joint),
        (spec.length - CAP_R, spec.length - CAP_R, right_joint),
    ):
        if not jointed:
            continue
        region: PartLike = Pos(region_lo, 0, 0) * Box(
            CAP_R, spec.width, spec.thickness, align=_XALIGN
        )
        roundoff: PartLike = Pos(pivot_x, 0, 0) * Cylinder(
            radius=CAP_R, height=spec.thickness, align=_ALIGN
        )
        corners: PartLike = region - roundoff
        part = _solid(part - corners + Pos(pivot_x, 0, 0) * _rivet(hole_r))

    # A jointed end needs MID_MARGIN clear of the Connector's own footprint.
    # A free end has no Connector there at all, so the thickened slab runs
    # flush all the way to the Segment's own tip edge (0 / spec.length) -
    # the whole free end, straight side AND round cap, ends up one uniform
    # thickness with nothing thin or stepped left exposed.
    mid_lo: float = (CAP_R + MID_MARGIN) if left_joint else 0.0
    mid_hi: float = (
        (spec.length - CAP_R - MID_MARGIN) if right_joint else spec.length
    )
    if mid_hi > mid_lo:
        thick_height: float = -CONNECTOR_BOTTOM_Z
        thick_align: _Align3 = (Align.MIN, Align.CENTER, Align.MAX)
        thick_box: Part = Box(
            mid_hi - mid_lo, spec.width, thick_height, align=thick_align
        )

        # Free ends: round the slab's own near-tip corners with the same
        # CAP_R fillet as the main body's tip, so the newly added material
        # follows the identical round outline instead of reintroducing a
        # square corner underneath an already-round tip.
        far_x: float = mid_hi - mid_lo
        tip_edges: list[Edge] = []
        if not left_joint:
            tip_edges += [
                e
                for e in thick_box.edges().filter_by(Axis.Z)
                if e.bounding_box().min.X < 1e-6
            ]
        if not right_joint:
            tip_edges += [
                e
                for e in thick_box.edges().filter_by(Axis.Z)
                if e.bounding_box().min.X > far_x - 1e-6
            ]
        thick: PartLike = (
            fillet(tip_edges, radius=_FREE_TIP_FILLET_R) if tip_edges else thick_box
        )

        # Jointed ends: the slab still stops MID_MARGIN short of the tip
        # (clearing the Connector), so its own near-tip wall still meets the
        # tip's flat underside in a sharp step - hidden under the Connector,
        # but still a real sharp edge in the mesh. Round the BOTTOM edge of
        # that wall (where it meets the slab's own flat underside, a convex
        # corner) - not the top edge (where it meets the tip's flat
        # underside, a concave corner from the wall's own perspective).
        step_z: float = -thick_height
        step_edges: list[Edge] = []
        if left_joint:
            step_edges += [
                e
                for e in thick.edges().filter_by(Axis.Y)
                if e.bounding_box().min.X < 1e-6
                and abs(e.bounding_box().min.Z - step_z) < 1e-6
            ]
        if right_joint:
            step_edges += [
                e
                for e in thick.edges().filter_by(Axis.Y)
                if e.bounding_box().min.X > far_x - 1e-6
                and abs(e.bounding_box().min.Z - step_z) < 1e-6
            ]
        step_r: float = min(STEP_ROUND_R, thick_height * 0.9, far_x * 0.9)
        if step_edges and step_r > 0:
            thick = fillet(step_edges, radius=step_r)

        thick = Pos(mid_lo, 0, 0) * thick
        part = _solid(part + thick)

    top: PartLike = _solid(chamfer(_rim_edges(part, top=True), EDGE_CHAMFER))
    bottom_edges: list[Edge] = _rim_edges(top, top=False) + _jointed_tip_bottom_rim_edges(
        top, spec, left_joint=left_joint, right_joint=right_joint
    )
    return _solid(fillet(bottom_edges, BOTTOM_FILLET_R))


def _connector() -> PartLike:
    """Stadium-shaped plate below the Segments. Each end is a stepped bore:
    a narrow collar bore (shaft clearance) on top, then a wide pocket
    (head clearance) that stays open at the Connector's own flat bottom
    face, with the rivet head recessed FLUSH_RECESS above that face."""
    hole_r: float = (PIN_D + PIN_CLEARANCE) / 2
    head_r: float = hole_r + HEAD_MARGIN
    pocket_r: float = head_r + POCKET_CLEARANCE
    # Exactly CONN_W / 2 (no inset margin): a true full-width semicircle,
    # tangent to the box's own flat side at the seam - same reasoning as
    # BAR_W for the Segment's tip cap. Previously inset, which left a sharp
    # vertical edge at the seam where the box's own square corner poked out
    # past the smaller cap curve.
    cap_r: float = CONN_W / 2

    top: float = -Z_GAP
    collar_bottom: float = top - CONNECTOR_T
    head_bottom: float = collar_bottom - HEAD_GAP - HEAD_T
    bottom: float = head_bottom - FLUSH_RECESS
    thickness: float = top - bottom

    part: PartLike = Pos(0, 0, top) * Box(
        HOLE_SPACING, CONN_W, thickness, align=_TOP_ALIGN
    )
    for x in (-HOLE_SPACING / 2, HOLE_SPACING / 2):
        cap: PartLike = Pos(x, 0, top) * Cylinder(
            radius=cap_r, height=thickness, align=_TOP_ALIGN
        )
        part = _solid(part + cap)
    for x in (-HOLE_SPACING / 2, HOLE_SPACING / 2):
        collar_bore: PartLike = Pos(x, 0, top) * Cylinder(
            radius=hole_r, height=top - collar_bottom, align=_TOP_ALIGN
        )
        pocket: PartLike = Pos(x, 0, collar_bottom) * Cylinder(
            radius=pocket_r, height=collar_bottom - bottom, align=_TOP_ALIGN
        )
        part = part - collar_bore - pocket
    return _chamfer_horizontal_rims(part, EDGE_CHAMFER)


def _segment_piece(
    spec: SegmentSpec, x: float, *, left_joint: bool, right_joint: bool
) -> PartLike:
    """A fully positioned Segment whose own box starts at `x` (its left edge,
    before any pivot inset) and runs `spec.length` further."""
    return Pos(x, 0, 0) * _segment(spec, left_joint=left_joint, right_joint=right_joint)


def _junction_piece(x: float) -> PartLike:
    """A fully positioned Connector centred at `x`, the seam between two
    touching Segment edges."""
    return Pos(x, 0, 0) * _connector()


def build_ruler(specs: Sequence[SegmentSpec] | None = None) -> PartLike:
    """Straight chain: one Segment per entry in `specs` (default: NUM_LINKS
    identical Segments), a Connector at every seam between them. Each
    Segment keeps exactly its own `spec.length` - the two free ends of the
    whole chain (first Segment's left tip, last Segment's right tip) are
    simply unjointed, not shortened or lengthened. A running cursor (not a
    fixed pitch) advances by each Segment's own length plus TIP_GAP, so
    Segments of different lengths still line up edge to edge."""
    if specs is None:
        specs = _default_specs()

    pieces: list[PartLike] = []
    cursor: float = 0.0
    for i, spec in enumerate(specs):
        left_joint: bool = i > 0
        right_joint: bool = i < len(specs) - 1
        pieces.append(
            _segment_piece(spec, cursor, left_joint=left_joint, right_joint=right_joint)
        )
        seg_end: float = cursor + spec.length
        if right_joint:
            pieces.append(_junction_piece(seg_end + TIP_GAP / 2))
            cursor = seg_end + TIP_GAP

    chain: PartLike = pieces[0]
    for piece in pieces[1:]:
        chain = _solid(chain + piece)
    return chain


def export_all(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    specs: list[SegmentSpec] = _default_specs()
    part: PartLike = build_ruler(specs)
    stl_path: Path = out_dir / "ruler.stl"
    step_path: Path = out_dir / "ruler.step"
    export_stl(part, str(stl_path))
    export_step(part, str(step_path))
    print(f"wrote {stl_path}")
    print(f"wrote {step_path}")
    print(f"extended length: {_total_length(specs):.1f}mm ({len(specs)} segments)")


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
