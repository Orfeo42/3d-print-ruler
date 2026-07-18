if __name__ == "__main__" and __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "articulated_ruler"

import math

from build123d import (
    CenterArc,
    Cylinder,
    Edge,
    GeomType,
    Line,
    Pos,
    Spline,
    Wire,
    extrude,
    fillet,
    make_face,
)

from .constants import (
    COLLAR_BEAD_FILLET_RADIUS,
    COLLAR_BEAD_RADIUS,
    COLLAR_GROOVE_RADIUS,
    COLLAR_HEIGHT,
    COLLAR_INNER_RADIUS,
    COLLAR_OUTER_RADIUS,
    COLLAR_TAB_ANCHOR_OVERREACH,
    COLLAR_TAB_BAND_THICKNESS,
    COLLAR_TAB_HALF_ANGLE_DEG,
    COLLAR_TAB_HEIGHT,
    COLLAR_TAB_RELIEF_GAP,
    COLLAR_TAB_RELIEF_HALF_ANGLE_DEG,
    COLLAR_TAB_TOP_GAP,
    COLLAR_TAB_Z_OFFSET,
    RECESS_HEIGHT,
    RECESS_INNER_RADIUS,
    RECESS_OUTER_RADIUS,
)
from .geometry import ALIGN_CENTER_BOTTOM, PartLike, require_solid


def build_recess() -> PartLike:
    """Segment-side void for the pivot collar - a full-circle annulus,
    centred on the local origin (the pivot). Full-circle, not limited to the
    fold's own angular range: the Connector's boss is a fixed full ring in
    the world frame, so any un-recessed Segment material at this radius
    band would rotate straight into it after only a few degrees of fold."""
    return require_solid(
        Cylinder(
            radius=RECESS_OUTER_RADIUS, height=RECESS_HEIGHT, align=ALIGN_CENTER_BOTTOM
        )
        - Cylinder(
            radius=RECESS_INNER_RADIUS, height=RECESS_HEIGHT, align=ALIGN_CENTER_BOTTOM
        )
    )


# Tab profile. The tab is a SPRING BAND hanging in the recess: a thin
# strip (COLLAR_TAB_BAND_THICKNESS) whose INNER edge is the working
# surface, anchored into solid Segment bulk only at its two angular ends,
# with an empty relief gap behind its mid-span (see build_tab_relief) and a
# top gap freeing its mid-span from the recess ceiling. The bead deflects
# the band outward and the band springs back - a real click from spring
# action, not bulk plastic compression (the rigid full-backed band
# print-tested with no felt lock: too stiff to click at any squeeze depth
# that still allowed rotation). Per side (mirrored about the socket's
# centre line, reading from the tab's angular end toward the centre): the
# inner edge starts fully open (flush with the anchor radius, zero reach -
# free entry), bulges progressively INWARD to a rounded squeeze peak that
# dips inside the bead's own swept reach, then releases into the socket's
# mouth on the groove circle itself.
COLLAR_TAB_SQUEEZE_ANGLE_DEG: float = 15.0  # where the squeeze bulge peaks (0 = socket centre line)
COLLAR_TAB_SQUEEZE_DEPTH: float = (
    0.25  # how far the bulge dips inside the bead's outer reach - now the
    # band's max spring deflection, not a crush depth. Was 0.4 on the rigid
    # band; on the spring 0.4 would overstrain PLA (~5%+ bending strain) -
    # this is THE click-strength knob: raise if the click is weak, lower if
    # the band cracks or the joint jams.
)
COLLAR_TAB_MOUTH_ANGLE_DEG: float = (
    75.0  # where on the groove circle the inner edge lands (0 = the
    # circle's outermost point). Also sets how far the socket's retaining
    # lips wrap around a seated bead: at 50 the lips only reached 0.13mm
    # inside the bead's swept radius (print-tested: no lock) - at 75 they
    # reach ~0.4mm in, a real wall the bead must be pushed back over to
    # unseat.
)
_FLANK_SAMPLES: int = 24


def _mouth_pivot_angle() -> float:
    """Pivot-frame angle (radians) of the point where the inner edge lands
    on the groove circle."""
    mouth: float = math.radians(COLLAR_TAB_MOUTH_ANGLE_DEG)
    mouth_x: float = COLLAR_OUTER_RADIUS + COLLAR_GROOVE_RADIUS * math.cos(mouth)
    mouth_y: float = COLLAR_GROOVE_RADIUS * math.sin(mouth)
    return math.atan2(mouth_y, mouth_x)


def _inner_radius_at(pivot_angle: float) -> float:
    """Radius of the tab's inner working edge along the ray at
    `pivot_angle` (radians, absolute value used - the edge is mirror
    symmetric). Piecewise: eased climb from the anchor radius to the
    squeeze peak, rounded release down to the mouth landing, then the far
    side of the groove circle itself across the socket's own span."""
    theta: float = abs(pivot_angle)
    wide_angle: float = math.radians(COLLAR_TAB_HALF_ANGLE_DEG)
    squeeze_angle: float = math.radians(COLLAR_TAB_SQUEEZE_ANGLE_DEG)
    anchor_radius: float = RECESS_OUTER_RADIUS + COLLAR_TAB_ANCHOR_OVERREACH
    squeeze_radius: float = (
        COLLAR_OUTER_RADIUS + COLLAR_BEAD_RADIUS - COLLAR_TAB_SQUEEZE_DEPTH
    )
    mouth_angle: float = _mouth_pivot_angle()
    if theta >= squeeze_angle:
        t: float = (theta - wide_angle) / (squeeze_angle - wide_angle)
        return anchor_radius + (squeeze_radius - anchor_radius) * t * (2 - t)
    if theta >= mouth_angle:
        mouth: float = math.radians(COLLAR_TAB_MOUTH_ANGLE_DEG)
        mouth_radius: float = math.hypot(
            COLLAR_OUTER_RADIUS + COLLAR_GROOVE_RADIUS * math.cos(mouth),
            COLLAR_GROOVE_RADIUS * math.sin(mouth),
        )
        t = (theta - squeeze_angle) / (mouth_angle - squeeze_angle)
        return squeeze_radius + (mouth_radius - squeeze_radius) * t * t
    half_chord: float = COLLAR_OUTER_RADIUS * math.sin(theta)
    return COLLAR_OUTER_RADIUS * math.cos(theta) + math.sqrt(
        COLLAR_GROOVE_RADIUS**2 - half_chord**2
    )


def _inner_edge_points() -> list[tuple[float, float]]:
    """XY samples for one side's inner working edge (positive-Y side), from
    the fully-open start at +COLLAR_TAB_HALF_ANGLE_DEG (zero inward reach)
    to the landing point on the groove circle."""
    wide_angle: float = math.radians(COLLAR_TAB_HALF_ANGLE_DEG)
    mouth_angle: float = _mouth_pivot_angle()
    points: list[tuple[float, float]] = []
    for i in range(2 * _FLANK_SAMPLES + 1):
        t: float = i / (2 * _FLANK_SAMPLES)
        angle: float = wide_angle + (mouth_angle - wide_angle) * t
        radius: float = _inner_radius_at(angle)
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _band_outer_points(half_angle_deg: float, extra_offset: float = 0.0) -> list[tuple[float, float]]:
    """XY samples for the band's outer edge (inner edge pushed out radially
    by the band thickness, plus `extra_offset`), swept continuously from
    -half_angle_deg to +half_angle_deg through the bulge behind the
    socket."""
    half: float = math.radians(half_angle_deg)
    offset: float = COLLAR_TAB_BAND_THICKNESS + extra_offset
    points: list[tuple[float, float]] = []
    for i in range(4 * _FLANK_SAMPLES + 1):
        angle: float = -half + 2 * half * i / (4 * _FLANK_SAMPLES)
        radius: float = _inner_radius_at(angle) + offset
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _top_gap_slab() -> PartLike:
    """Cutter freeing the band's mid-span top face from the recess ceiling:
    an angular sector spanning the relief range, radially generous (it is
    only ever subtracted from the tab, never from the Segment), covering
    the top COLLAR_TAB_TOP_GAP of the band's height."""
    half: float = math.radians(COLLAR_TAB_RELIEF_HALF_ANGLE_DEG)
    inner_r: float = COLLAR_INNER_RADIUS
    outer_r: float = RECESS_OUTER_RADIUS + COLLAR_TAB_ANCHOR_OVERREACH + 1.0
    outer_arc: Edge = CenterArc(
        (0, 0),
        outer_r,
        -COLLAR_TAB_RELIEF_HALF_ANGLE_DEG,
        2 * COLLAR_TAB_RELIEF_HALF_ANGLE_DEG,
    )
    inner_arc: Edge = CenterArc(
        (0, 0),
        inner_r,
        COLLAR_TAB_RELIEF_HALF_ANGLE_DEG,
        -2 * COLLAR_TAB_RELIEF_HALF_ANGLE_DEG,
    )
    side_pos: Edge = Line(
        (outer_r * math.cos(half), outer_r * math.sin(half)),
        (inner_r * math.cos(half), inner_r * math.sin(half)),
    )
    side_neg: Edge = Line(
        (inner_r * math.cos(half), -inner_r * math.sin(half)),
        (outer_r * math.cos(half), -outer_r * math.sin(half)),
    )
    face = make_face(Wire([outer_arc, side_pos, inner_arc, side_neg]))
    slab: PartLike = require_solid(extrude(face, amount=COLLAR_TAB_TOP_GAP + 1.0))
    if slab.bounding_box().max.Z < COLLAR_TAB_TOP_GAP / 2:
        slab = require_solid(extrude(face, amount=-(COLLAR_TAB_TOP_GAP + 1.0)))
    return Pos(0, 0, COLLAR_TAB_HEIGHT - COLLAR_TAB_TOP_GAP) * slab


def build_tab() -> PartLike:
    """Segment-side catch living inside the recess void, centred on the
    pivot: a thin spring band. Its outer edge is the inner working edge
    pushed out by COLLAR_TAB_BAND_THICKNESS; at the angular ends that outer
    edge pokes past RECESS_OUTER_RADIUS into solid Segment material, which
    is the band's only anchoring - mid-span it floats free (relief gap
    behind it via build_tab_relief, top gap above via _top_gap_slab) so the
    bead can bow it outward and it springs back. Per side (mirrored about
    the socket's centre line), the inner edge starts fully open at the
    angular end, bulges inward to a rounded squeeze peak dipping
    COLLAR_TAB_SQUEEZE_DEPTH inside the bead's own swept reach, then
    releases onto the groove circle itself - so the bead enters free,
    deflects the band, and drops into its seat with a click. The socket
    matches the boss's own bead cylinders: COLLAR_GROOVE_RADIUS = bead
    radius plus seating clearance, centred at the same COLLAR_OUTER_RADIUS
    the beads sit on."""
    inner_edge: list[tuple[float, float]] = _inner_edge_points()
    outer_edge: list[tuple[float, float]] = _band_outer_points(COLLAR_TAB_HALF_ANGLE_DEG)

    inner_pos: Edge = Spline(*inner_edge)
    mouth_arc_pos: Edge = CenterArc(
        (COLLAR_OUTER_RADIUS, 0),
        COLLAR_GROOVE_RADIUS,
        COLLAR_TAB_MOUTH_ANGLE_DEG,
        -COLLAR_TAB_MOUTH_ANGLE_DEG,
    )
    mouth_arc_neg: Edge = CenterArc(
        (COLLAR_OUTER_RADIUS, 0),
        COLLAR_GROOVE_RADIUS,
        0,
        -COLLAR_TAB_MOUTH_ANGLE_DEG,
    )
    inner_neg: Edge = Spline(*[(x, -y) for x, y in reversed(inner_edge)])
    end_neg: Edge = Line((inner_edge[0][0], -inner_edge[0][1]), outer_edge[0])
    outer: Edge = Spline(*outer_edge)
    end_pos: Edge = Line(outer_edge[-1], inner_edge[0])

    wire = Wire([inner_pos, mouth_arc_pos, mouth_arc_neg, inner_neg, end_neg, outer, end_pos])
    face = make_face(wire)
    # The extrude direction depends on the wire's winding (verified bug: a
    # flipped winding once put the whole tab below Z=0, silently missing
    # the groove cut) - guard on the actual result instead of trusting it.
    # Threshold is half the height, not a bare epsilon: OCCT bounding boxes
    # carry ~1e-7 slop below Z=0 even for a correct upward extrude, which a
    # tight epsilon misreads as "went down" (verified - it flipped a correct
    # extrude and reintroduced the very bug this guard exists to catch).
    tab_solid: PartLike = require_solid(extrude(face, amount=COLLAR_TAB_HEIGHT))
    if tab_solid.bounding_box().max.Z < COLLAR_TAB_HEIGHT / 2:
        tab_solid = require_solid(extrude(face, amount=-COLLAR_TAB_HEIGHT))
    tab: PartLike = Pos(0, 0, COLLAR_TAB_Z_OFFSET) * tab_solid
    # No rim fillet on the spring band: on this thin, slab-stepped shape the
    # 0.05 rim fillet "succeeds" as an OCCT op but exports a non-watertight
    # mesh and makes the later Segment fuse return a Null shape (verified
    # stage-by-stage). 0.05 is below FDM resolution anyway.
    tab = require_solid(tab - _top_gap_slab())

    # The socket cut proper - same cylinder shape as the boss's beads,
    # rounded before subtracting (the tool's own rims, not the tab's) so
    # both openings get a smooth funnel lip.
    groove_cylinder: PartLike = Cylinder(
        radius=COLLAR_GROOVE_RADIUS, height=COLLAR_TAB_HEIGHT, align=ALIGN_CENTER_BOTTOM
    )
    groove_cylinder = fillet(
        groove_cylinder.edges().filter_by(GeomType.CIRCLE),
        radius=COLLAR_BEAD_FILLET_RADIUS,
    )
    groove: PartLike = (
        Pos(COLLAR_OUTER_RADIUS, 0, COLLAR_TAB_Z_OFFSET) * groove_cylinder
    )
    return require_solid(tab - groove)


# The relief void's inner wall reaches slightly INSIDE the band's outer
# surface instead of landing exactly on it: both are Splines sampled from
# the same profile but with different parametrizations, so "exactly on it"
# is really a micron-scale wobble crossing to both sides - and unioning the
# tab against that near-coincident wall makes the fuse come back INVALID
# ("Boolean operation unable to clean", verified stage-by-stage). The tab
# union afterwards restores the eps of band material, leaving a genuine
# volumetric overlap for the fuse to chew on.
_RELIEF_OVERLAP_EPS: float = 0.02


def build_tab_relief() -> PartLike:
    """Void carved out of the Segment BEHIND the band's mid-span (subtract
    from the Segment before unioning the tab in): the region between the
    band's outer edge and that edge pushed out a further
    COLLAR_TAB_RELIEF_GAP, across the relief angular span only - the band's
    angular ends past that span stay embedded in solid Segment material and
    are its anchors. Without this void the band has bulk right behind it
    and cannot bow outward at all."""
    inner_pts: list[tuple[float, float]] = _band_outer_points(
        COLLAR_TAB_RELIEF_HALF_ANGLE_DEG, extra_offset=-_RELIEF_OVERLAP_EPS
    )
    outer_pts: list[tuple[float, float]] = _band_outer_points(
        COLLAR_TAB_RELIEF_HALF_ANGLE_DEG, extra_offset=COLLAR_TAB_RELIEF_GAP
    )
    inner: Edge = Spline(*inner_pts)
    end_pos: Edge = Line(inner_pts[-1], outer_pts[-1])
    outer: Edge = Spline(*list(reversed(outer_pts)))
    end_neg: Edge = Line(outer_pts[0], inner_pts[0])
    face = make_face(Wire([inner, end_pos, outer, end_neg]))
    relief: PartLike = require_solid(extrude(face, amount=RECESS_HEIGHT))
    if relief.bounding_box().max.Z < RECESS_HEIGHT / 2:
        relief = require_solid(extrude(face, amount=-RECESS_HEIGHT))
    return relief


def build_boss(target_angles_deg: tuple[float, float]) -> PartLike:
    """Connector-side raised ring, centred on the local origin (the pivot),
    with a bead protruding from its own wall at each target fold angle -
    see `build_tab` for the matching catch."""
    boss: PartLike = require_solid(
        Cylinder(
            radius=COLLAR_OUTER_RADIUS, height=COLLAR_HEIGHT, align=ALIGN_CENTER_BOTTOM
        )
        - Cylinder(
            radius=COLLAR_INNER_RADIUS, height=COLLAR_HEIGHT, align=ALIGN_CENTER_BOTTOM
        )
    )
    for target_deg in target_angles_deg:
        rad: float = math.radians(target_deg)
        bead_x: float = COLLAR_OUTER_RADIUS * math.cos(rad)
        bead_y: float = COLLAR_OUTER_RADIUS * math.sin(rad)
        bead_cylinder: PartLike = Cylinder(
            radius=COLLAR_BEAD_RADIUS, height=COLLAR_HEIGHT, align=ALIGN_CENTER_BOTTOM
        )
        bead_cylinder = fillet(
            bead_cylinder.edges().filter_by(GeomType.CIRCLE),
            radius=COLLAR_BEAD_FILLET_RADIUS,
        )
        bead: PartLike = Pos(bead_x, bead_y, 0) * bead_cylinder
        boss = require_solid(boss + bead)
    return boss


if __name__ == "__main__":
    from .constants import FOLD_CLOSED_ANGLE_DEG

    tab = build_tab()
    boss = build_boss((FOLD_CLOSED_ANGLE_DEG, -FOLD_CLOSED_ANGLE_DEG))
    try:
        from ocp_vscode import show

        show(tab, boss, names=["tab", "boss"])
    except Exception as exc:
        print(f"preview skipped ({type(exc).__name__}: {exc})")
