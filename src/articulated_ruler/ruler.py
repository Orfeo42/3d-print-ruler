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

import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from build123d import (
    Align,
    Axis,
    Box,
    Compound,
    Curve,
    Cylinder,
    Edge,
    GeomType,
    Part,
    Polygon,
    Pos,
    Wire,
    chamfer,
    export_step,
    export_stl,
    extrude,
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
# Widened from 8.0mm to fit the pivot collar's catch bump (see COLLAR_* below)
# with real safety margin inside the Connector's own edge - the original
# 8.0mm left only ~1mm of radial room total between the rivet's own pocket
# and the Connector's edge, not enough for a boss ring AND a nozzle-safe
# bump AND clearance margins on both sides at once.
CONN_W: Final[float] = 9.0  # Connector plate width

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

# The ruler's "closed" fold angle: how far one Segment rotates about its own
# pivot, relative to its fixed neighbour+Connector, before "side by side,
# long edges touching" (measured via an ad hoc collision sweep - rotate one
# Segment against a fixed neighbour+Connector, find the first angle with
# non-zero exact intersection volume - see DEVELOPMENT.md; came out to
# ~93 degrees on this geometry). FOLD_CLOSED_DEG sits a few degrees short of
# that measured limit, for tolerance margin.
#
# See DEVELOPMENT.md's "Snap catch for a fully-closed ruler" section for
# three tried-and-reverted attempts (pivot-bore ball detent; Connector-edge
# flex tab; a cam bump/tooth pair crammed inside the ~2mm pivot bore, in
# three sizing passes) before the current one below.
FOLD_CLOSED_DEG: Final[float] = 90.0

# Rivet head's own pocket radius - reused below since the new pivot collar
# (COLLAR_R_IN) has to clear it too, not just _connector's own local pocket.
POCKET_R: Final[float] = (PIN_D + PIN_CLEARANCE) / 2 + HEAD_MARGIN + POCKET_CLEARANCE

# Snap catch, attempt 5: a dedicated "pivot collar" - a raised boss on the
# Connector's top face, around each hole, sitting in all-new Z headroom
# (COLLAR_HEIGHT) rather than reusing the cramped pivot bore or the fold's
# own collision envelope like every prior attempt. Every earlier attempt
# tried to fit a catch feature into a space that was never big enough for
# one (a ~2mm bore radius, or the 0.15mm Z_GAP) - this instead opens real
# room for it, sized in whole millimetres rather than fractions of the
# nozzle width. Two matching pieces:
#   - Connector: a full-circle boss, radius COLLAR_R_IN to COLLAR_R_OUT,
#     height COLLAR_HEIGHT, with a small BUMP protruding OUTWARD from its
#     wall at each of the two "closed" world angles (+-FOLD_CLOSED_DEG).
#   - Segment: a matching full-circle recess cut into its own underside
#     (same radius band, so the boss - being a fixed full ring in the world
#     frame - never collides with un-recessed Segment material at ANY fold
#     angle; a partial-arc recess was tried first and rejected on paper -
#     any un-recessed "back" material at this radius band rotates straight
#     into the boss's own footprint after only a few degrees of fold, since
#     the boss occupies every world angle), housing a cantilevered flex tab.
#     The tab is a thin arc-shaped rib running tangentially - most of its
#     length hangs free inside the recess, with a small radial "root" at one
#     end poking OUTWARD past the recess's own outer radius (RECESS_R) to
#     fuse with solid, un-recessed Segment material just beyond it - a
#     radius the boss never reaches, so this anchor never collides with
#     anything regardless of fold angle. Free at the other end, at local
#     angle 0 (the assembly's own rest orientation). A radial push from the
#     boss's bump is then a transverse (bending) load on this tangential
#     beam, not an axial one a beam barely deflects under.
#
# First version of this used a constant PRELOAD instead (tab's nominal
# reach always poking slightly past the boss's plain wall, relaxing into a
# NOTCH at the catch position) - reverted after export showed 1 fused
# watertight body instead of 5. Root cause: a static boolean model has no
# concept of "meant to flex apart" - any interference present in the
# AS-BUILT (rest) orientation is just permanent geometric overlap to a
# straightforward union, identical to attempt 1's own fusion failure. The
# fix is the same lesson attempt 1 eventually needed: ZERO interference at
# rest and through the free-fold range, with a real, localized BUMP that
# only reaches into contact once the tab (rotating with the Segment) sweeps
# near +-FOLD_CLOSED_DEG - the same "matched asymmetric feature pair"
# topology as the reverted cam-bump-in-bore attempt, just relocated to a
# radius/arc scale big enough to print reliably. Fitting the bump without
# it poking past the Connector's own edge needed CONN_W widened too (see
# above) - the original 8mm width left under 1mm of total radial room
# between the rivet's pocket and the Connector's edge, not enough for a
# boss ring AND a nozzle-safe bump AND clearance margins on both sides.
# Reversible: the bump's own sides are symmetric, so the tab can be pushed
# back off it by hand, unlike a one-way ratchet ramp.
COLLAR_R_IN: Final[float] = POCKET_R + 0.15  # clears the rivet's own pocket wall
COLLAR_HEIGHT: Final[float] = 1.2  # dedicated Z headroom, above the Connector's existing top
COLLAR_CLEARANCE: Final[float] = 0.2  # Z clearance, boss top to the Segment's recess ceiling
COLLAR_RADIAL_CLEARANCE: Final[float] = 0.15  # radial clearance, boss walls to recess walls

# Attempt 5.1 used a flat-topped radial WEDGE for the bump (sharp-cornered,
# rectangular in cross-section) - per the user's own explicit correction,
# not what was actually wanted: "a semicylinder bump and a same size
# cylinder indent". Both are now a plain vertical Cylinder, not a wedge:
# - Connector (the boss, "the circle"): a small cylinder centred exactly
#   ON the boss's own outer wall (radius COLLAR_R_OUT) - since its centre
#   sits right at that radius, half of it (the outer half) protrudes past
#   the wall as a true semicylinder bead (flat side flush with the wall,
#   round side outward), the other half is embedded in the boss's own
#   material for a clean union.
# - Segment (the tab, "the external piece"): a matching, slightly larger
#   cylinder cut INTO the tab's own free tip (local angle 0 - the same
#   point that lands exactly on a bump's own position once the joint
#   reaches +-FOLD_CLOSED_DEG) - a same-size round indent, not a flat
#   notch.
# Mechanically unchanged from 5.1's own reasoning: away from the bump, the
# tab's nominal clearance means no contact at all; sweeping toward
# +-FOLD_CLOSED_DEG, the bead (reaching further out than that nominal
# clearance) pushes the tab out; at exactly the closed angle, the tab's own
# round indent lands on the bead and lets it seat, releasing the tab back
# toward rest - the actual push-then-release click, now with a rounded
# profile that seats more smoothly than the old wedge's sharp corners.
# Reversible either way (both are round, no one-way ratchet face).
COLLAR_BEAD_R: Final[float] = (
    0.65  # bead cylinder radius - bumped from 0.5 (attempt 6) for a bigger, more
    # resolvable, more strongly-felt catch ("not that snappy / not strong as
    # a block" feedback on the smaller bead) - the WHOLE radius protrudes,
    # since the bead's own centre sits ON the wall (see above)
)
COLLAR_GROOVE_R: Final[float] = (
    COLLAR_BEAD_R + 0.1  # the Segment's matching indent - a little larger than the bead for a free, non-interference seat once aligned. Kept a real margin short of the tab's own outer wall (COLLAR_R_OUT + COLLAR_TAB_CLEARANCE + COLLAR_TAB_THICKNESS) - an earlier value put the groove's own outer edge EXACTLY there (an unintended tangency), which left a sliver of tab material pinched off as its own disconnected body (caught by the watertight-body count)
)
# Rounds the bead's own top/bottom rim (softer, less snag-prone contact
# against the tab) and the groove's own top/bottom rim (a small funnel
# easing the bead in and out) - small, since both features are themselves
# only ~1mm across, but a real, deliberate break rather than a hard
# printed corner on the two surfaces that actually rub against each other.
COLLAR_BEAD_FILLET_R: Final[float] = 0.15
# The tab's own rim fillet - this new rigid V-wedge shape (see
# `_pivot_collar_tab`) has none of the earlier flex-beam's fragile-bridge
# issue (its groove sits in the middle of a solid mass reaching all the way
# to its own peak/anchor, not a thin cantilever relying on a sliver of
# material), so one plain radius on the whole rim is enough.
COLLAR_TAB_FILLET_R: Final[float] = (
    0.05  # 0.15 fails outright on this shape's sharp apex vertex; 0.08-0.12
    # "succeed" as a boolean op but mesh into 40x the face count (near-
    # degenerate slivers at the acute apex angle) and broke watertightness
    # once combined with the rest of the assembly - 0.05 meshes clean
    # (~2000 faces, matching the un-filleted face count) and stays
    # watertight end to end
)
# How many straight segments approximate the flush arc against the boss's
# own wall - fine enough to read as smooth, coarse enough to keep the
# `Polygon` simple.
_TAB_ARC_SEGMENTS: Final[int] = 12

# Stays inside the Connector's own cap edge (CONN_W / 2) even with the bead
# at its biggest reach, plus a real safety margin.
COLLAR_R_OUT: Final[float] = CONN_W / 2 - COLLAR_BEAD_R - 0.2

# Attempt 7 tried a symmetric FIXED-FIXED beam (anchored both ends, groove
# in the middle) so the SAME joint could click from either rotation
# direction, not just attempt 6's single-anchor cantilever. Proven
# geometrically impossible for this bead layout: a boss carries TWO fixed
# beads (+-FOLD_CLOSED_DEG), and ANY rigid anchor close enough to the
# groove to matter, at local angle `a` on one side, sweeps through world
# angle 90 (or -90) at rotation = 90-a for SOME rotation within the
# achievable 0..~93.5-degree range whenever `a` is on the SAME side as that
# bead - verified directly (not just derived): even attempt 6's own
# original single anchor (local -50 to -35) rigidly collides with the
# OTHER (-90) bead across roughly -45 to -75 degrees of rotation the
# "wrong" way (constant ~0.165mm3 overlap - a hard block, not a spring).
# Per explicit user direction, keeping BOTH beads (for whichever future
# joint arrangement needs the other one) and accepting that folding a
# given joint backward blocks rather than clicks.
#
# Attempt 9 (current): per an explicit follow-up request, dropped the
# calculated-flex-cantilever idea (attempts 6-8) entirely in favour of a
# solid, RIGID catch - no clearance gap to the boss's own wall at all
# ("completely to the wall"), and half the previous angular reach. See
# `_pivot_collar_tab`'s own docstring for the resulting V-shaped-wedge
# construction. Skipped re-deriving the rotation sweep/strain numbers this
# pass, per that same request.
COLLAR_TAB_ARC_DEG: Final[float] = 20.0  # halved from 40 - "no need that wide"
COLLAR_TAB_THICKNESS: Final[float] = (
    1.0  # only used to size RECESS_R's own outer bound now - the tab's actual shape no longer has a single constant thickness (see _pivot_collar_tab)
)
COLLAR_TAB_CLEARANCE: Final[float] = (
    0.05  # was a deliberate flex standoff (0.2mm) - now down to a bare
    # minimum: reads as rigid/flush ("completely to the wall") while
    # staying a REAL, nonzero gap. True zero was tried first and reverted
    # - it doesn't just mean "tight," it means the boolean union actually
    # FUSES the segment and connector into one solid at the tab (verified
    # directly: `seg + conn` came back as 1 solid, not 2, with genuine
    # nonzero intersection volume), permanently welding that joint shut
    # rather than just making it stiff.
)
COLLAR_TAB_HEIGHT: Final[float] = 0.8  # tab's own Z extent, within the recess
# Kept off Z=0 on purpose: `_jointed_tip_bottom_rim_edges` fillets any flat
# face it finds sitting exactly at Z=0 (the Segment's own bottom plane) at
# BOTTOM_FILLET_R - a tab/root starting exactly there got its own thin
# bottom edges swept into that same 0.4mm fillet and failed outright
# (insufficient material at the tab's own 0.7mm thickness). A small lift
# keeps the tab's own faces off that plane entirely, while staying well
# within the boss's own Z range for genuine overlap.
COLLAR_TAB_Z_LO: Final[float] = 0.1
COLLAR_TAB_ROOT_OVERREACH: Final[float] = (
    0.15  # how far the tab's own peak pokes past RECESS_R, into solid Segment material - just enough for a clean union (it's its own anchor now, no separate root piece)
)

RECESS_R_IN: Final[float] = COLLAR_R_IN - COLLAR_RADIAL_CLEARANCE
# Clears the tab's own nominal outer reach (COLLAR_R_OUT + COLLAR_TAB_CLEARANCE
# + COLLAR_TAB_THICKNESS), not just the boss's plain wall.
RECESS_R: Final[float] = (
    COLLAR_R_OUT + COLLAR_TAB_CLEARANCE + COLLAR_TAB_THICKNESS + COLLAR_RADIAL_CLEARANCE
)
RECESS_CEIL_Z: Final[float] = -Z_GAP + COLLAR_HEIGHT + COLLAR_CLEARANCE

NUM_LINKS: Final[int] = 3

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


def _pivot_collar_recess() -> PartLike:
    """The Segment-side void for the new pivot collar (see COLLAR_* /
    FOLD_CLOSED_DEG) - a full-circle annulus, centred on the local origin
    (the pivot). Full-circle, not limited to the fold's own angular range:
    the Connector's boss is a fixed full ring in the world frame, so any
    un-recessed Segment material at this radius band would rotate straight
    into it after only a few degrees of fold."""
    return _solid(
        Cylinder(radius=RECESS_R, height=RECESS_CEIL_Z, align=_ALIGN)
        - Cylinder(radius=RECESS_R_IN, height=RECESS_CEIL_Z, align=_ALIGN)
    )


def _pivot_collar_tab() -> PartLike:
    """The Segment-side catch, living inside `_pivot_collar_recess`'s own
    void, centred on the local origin (the pivot). No longer a flex
    cantilever (attempts 6-8) - per explicit request, now a solid, RIGID
    V-shaped wedge: flush against the boss's own wall (COLLAR_TAB_CLEARANCE
    is a bare, near-zero 0.05mm - reads as rigid/flush ("completely to the
    wall"), not a spring standoff, but stays a REAL gap: true zero was
    tried first and reverted, since it doesn't just mean "tight" - it
    makes the boolean union actually FUSE the Segment and Connector into
    one solid at the tab, welding that joint shut instead of just making
    it stiff) at its two angular ends, rising in a straight line (the "V")
    to a peak that reaches past the recess's own outer radius and fuses
    directly with solid, un-recessed Segment material there - its own peak
    IS its anchor, no separate root piece needed. The round groove is cut
    through that peak, same as every prior attempt.

    Built as a `Polygon`: an inner boundary of points along the boss's own
    circle (radius COLLAR_R_OUT + COLLAR_TAB_CLEARANCE - approximated by
    `_TAB_ARC_SEGMENTS` short straight segments, fine enough to read as a
    smooth curve) from -COLLAR_TAB_ARC_DEG to +COLLAR_TAB_ARC_DEG, plus one
    apex point beyond RECESS_R - simpler and far more robust than deriving
    rotated half-space cuts for a line that doesn't pass through the
    origin."""
    half_angle: float = math.radians(COLLAR_TAB_ARC_DEG)
    apex_r: float = RECESS_R + COLLAR_TAB_ROOT_OVERREACH
    tab_r_lo: float = COLLAR_R_OUT + COLLAR_TAB_CLEARANCE
    arc_pts: list[tuple[float, float]] = [
        (
            tab_r_lo * math.cos(-half_angle + 2 * half_angle * i / _TAB_ARC_SEGMENTS),
            tab_r_lo * math.sin(-half_angle + 2 * half_angle * i / _TAB_ARC_SEGMENTS),
        )
        for i in range(_TAB_ARC_SEGMENTS + 1)
    ]
    sketch = Polygon(*arc_pts, (apex_r, 0.0))
    tab: PartLike = Pos(0, 0, COLLAR_TAB_Z_LO) * _solid(extrude(sketch, amount=-COLLAR_TAB_HEIGHT))
    # Round every corner - the flush arc's own short facet joints, the two
    # straight "V" ramps, and the peak - via the top/bottom rim, same
    # fillet-before-groove-cut approach as before. This shape has none of
    # the earlier fragile-bridge issue (the groove now sits in the middle
    # of a solid mass reaching all the way to the peak, not a thin
    # cantilever relying on a sliver), so one plain fillet call on the
    # whole rim is enough.
    tab_rim: list[Edge] = _rim_edges(tab, top=True) + _rim_edges(tab, top=False)
    tab = fillet(tab_rim, COLLAR_TAB_FILLET_R)
    # The matching round indent - centred at the SAME radius as the bead's
    # own centre (COLLAR_R_OUT), at local angle 0 (the peak - the point
    # that lands exactly on a bead's own position once the joint reaches
    # FOLD_CLOSED_DEG). Rounded before subtracting - the tool cylinder's
    # own rim, not the tab's - so the cut leaves a smooth funnel at both
    # openings instead of a sharp 90-degree step, the same "round before
    # combining" logic as the bead above.
    groove_cyl: PartLike = Cylinder(
        radius=COLLAR_GROOVE_R, height=COLLAR_TAB_HEIGHT, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    groove_cyl = fillet(
        groove_cyl.edges().filter_by(GeomType.CIRCLE), radius=COLLAR_BEAD_FILLET_R
    )
    groove: PartLike = Pos(COLLAR_R_OUT, 0, COLLAR_TAB_Z_LO) * groove_cyl
    return _solid(tab - groove)


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
    return _solid(shaft + head)


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
        part = _solid(
            part
            - Pos(pivot_x, 0, 0) * _pivot_collar_recess()
            + Pos(pivot_x, 0, 0) * _pivot_collar_tab()
        )

    # A jointed end needs MID_MARGIN clear of the Connector's own footprint.
    # A free end has no Connector there at all, so the thickened slab runs
    # flush all the way to the Segment's own tip edge (0 / spec.length) -
    # the whole free end, straight side AND round cap, ends up one uniform
    # thickness with nothing thin or stepped left exposed.
    mid_lo: float = (CAP_R + MID_MARGIN) if left_joint else 0.0
    mid_hi: float = (spec.length - CAP_R - MID_MARGIN) if right_joint else spec.length
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
    bottom_edges: list[Edge] = _rim_edges(
        top, top=False
    ) + _jointed_tip_bottom_rim_edges(
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
    part = _chamfer_horizontal_rims(_solid(part), EDGE_CHAMFER)

    # The pivot collar boss is added AFTER the rim chamfer, not before: it
    # rises above the Connector's own original top surface, so if it were
    # present during `_chamfer_horizontal_rims`, `_rim_edges` would pick the
    # boss's own tiny top ring as the Connector's "top face" instead of its
    # real outer silhouette (`faces().sort_by(Axis.Z)` just takes the
    # Z-highest face) - wrong edges entirely, and the chamfer op failed
    # outright against the boss's own tight ring geometry besides. The boss
    # itself doesn't need a rim chamfer - it's a functional catch feature,
    # not a user-facing edge.
    for x in (-HOLE_SPACING / 2, HOLE_SPACING / 2):
        boss: PartLike = _solid(
            Pos(x, 0, top) * Cylinder(radius=COLLAR_R_OUT, height=COLLAR_HEIGHT, align=_ALIGN)
            - Pos(x, 0, top) * Cylinder(radius=COLLAR_R_IN, height=COLLAR_HEIGHT, align=_ALIGN)
        )
        for target_deg in (FOLD_CLOSED_DEG, -FOLD_CLOSED_DEG):
            rad: float = math.radians(target_deg)
            bead_x: float = x + COLLAR_R_OUT * math.cos(rad)
            bead_y: float = COLLAR_R_OUT * math.sin(rad)
            bead_cyl: PartLike = Cylinder(
                radius=COLLAR_BEAD_R, height=COLLAR_HEIGHT, align=_ALIGN
            )
            # Rounded before placing/unioning, not after: once fused with
            # the boss, only the OUTER (protruding) arc of this rim survives
            # as a real edge - the rest is absorbed into the boss's own flat
            # top/bottom, so filleting the standalone cylinder first lands
            # the round-over exactly on the surface that actually rubs
            # against the tab.
            bead_cyl = fillet(
                bead_cyl.edges().filter_by(GeomType.CIRCLE), radius=COLLAR_BEAD_FILLET_R
            )
            bead: PartLike = Pos(bead_x, bead_y, top) * bead_cyl
            boss = _solid(boss + bead)
        part = _solid(part + boss)
    return part


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
