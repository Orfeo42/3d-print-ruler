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
    COLLAR_TAB_FILLET_RADIUS,
    COLLAR_TAB_HALF_ANGLE_DEG,
    COLLAR_TAB_HEIGHT,
    COLLAR_TAB_Z_OFFSET,
    RECESS_HEIGHT,
    RECESS_INNER_RADIUS,
    RECESS_OUTER_RADIUS,
)
from .geometry import ALIGN_CENTER_BOTTOM, PartLike, outer_rim_edges, require_solid


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


# Tab profile. The tab HANGS FROM THE OUTSIDE: a solid band anchored along
# the recess's own outer rim (fused into solid Segment material there),
# whose INNER edge is the working surface. The channel between that inner
# edge and the boss's own wall is where the bead travels. Per side
# (mirrored about the socket's centre line, reading from the tab's angular
# end toward the centre): the inner edge starts fully open (flush with the
# anchor band, zero reach - free entry), bulges progressively INWARD to a
# rounded squeeze peak that dips inside the bead's own swept reach, then
# releases into the socket's mouth on the groove circle itself.
COLLAR_TAB_SQUEEZE_ANGLE_DEG: float = (
    15.0  # where the squeeze bulge peaks (0 = socket centre line)
)
COLLAR_TAB_SQUEEZE_DEPTH: float = (
    0.5  # how far the bulge dips inside the bead's outer reach - the actual
    # snap interference and THE click-strength knob. 0.25 print-tested with
    # no felt lock; raise if the click is weak, lower if the joint jams.
)
COLLAR_TAB_MOUTH_ANGLE_DEG: float = (
    75.0  # where on the groove circle the inner edge lands (0 = the
    # circle's outermost point). Also sets how far the socket's retaining
    # lips wrap around a seated bead: at 50 the lips only reached 0.13mm
    # inside the bead's swept radius - under one nozzle width, the slicer
    # erased them (print-tested: no lock). At 75 they reach ~0.39mm in, a
    # real wall the bead must be pushed back over to unseat.
)
_FLANK_SAMPLES: int = 24


def _inner_edge_points() -> list[tuple[float, float]]:
    """XY samples for one side's inner working edge (positive-Y side), from
    the fully-open start at +COLLAR_TAB_HALF_ANGLE_DEG (at the outer anchor
    band, zero inward reach) to the landing point on the groove circle.
    Inward-reach shape: biggest slope at the start easing to a rounded
    maximum at the squeeze peak, then a rounded release into the mouth."""
    wide_angle: float = math.radians(COLLAR_TAB_HALF_ANGLE_DEG)
    squeeze_angle: float = math.radians(COLLAR_TAB_SQUEEZE_ANGLE_DEG)
    anchor_radius: float = RECESS_OUTER_RADIUS + COLLAR_TAB_ANCHOR_OVERREACH
    squeeze_radius: float = (
        COLLAR_OUTER_RADIUS + COLLAR_BEAD_RADIUS - COLLAR_TAB_SQUEEZE_DEPTH
    )
    mouth: float = math.radians(COLLAR_TAB_MOUTH_ANGLE_DEG)
    mouth_x: float = COLLAR_OUTER_RADIUS + COLLAR_GROOVE_RADIUS * math.cos(mouth)
    mouth_y: float = COLLAR_GROOVE_RADIUS * math.sin(mouth)
    mouth_angle: float = math.atan2(mouth_y, mouth_x)
    mouth_radius: float = math.hypot(mouth_x, mouth_y)

    points: list[tuple[float, float]] = []
    for i in range(_FLANK_SAMPLES + 1):
        t: float = i / _FLANK_SAMPLES
        angle: float = wide_angle + (squeeze_angle - wide_angle) * t
        radius: float = anchor_radius + (squeeze_radius - anchor_radius) * t * (2 - t)
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    for i in range(1, _FLANK_SAMPLES + 1):
        t = i / _FLANK_SAMPLES
        angle = squeeze_angle + (mouth_angle - squeeze_angle) * t
        radius = squeeze_radius + (mouth_radius - squeeze_radius) * t * t
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def build_tab() -> PartLike:
    """Segment-side catch living inside the recess void, centred on the
    pivot. A solid band anchored along the recess's outer rim (poking
    COLLAR_TAB_ANCHOR_OVERREACH past it, fusing with solid Segment material
    along its whole span), leaving an open channel along the boss's own
    wall for the bead to travel in. Per side (mirrored about the socket's
    centre line), the band's inner edge starts fully open at the angular
    end, bulges progressively inward to a rounded squeeze peak dipping
    COLLAR_TAB_SQUEEZE_DEPTH inside the bead's own swept reach, then
    releases onto the groove circle itself - so the bead enters free, gets
    squeezed harder and harder, and drops into its seat. The socket matches
    the boss's own bead cylinders: COLLAR_GROOVE_RADIUS = bead radius plus
    seating clearance, centred at the same COLLAR_OUTER_RADIUS the beads
    sit on."""
    anchor_radius: float = RECESS_OUTER_RADIUS + COLLAR_TAB_ANCHOR_OVERREACH
    inner_edge: list[tuple[float, float]] = _inner_edge_points()

    anchor_arc: Edge = CenterArc(
        (0, 0), anchor_radius, -COLLAR_TAB_HALF_ANGLE_DEG, 2 * COLLAR_TAB_HALF_ANGLE_DEG
    )
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

    wire = Wire([anchor_arc, inner_pos, mouth_arc_pos, mouth_arc_neg, inner_neg])
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

    tab_rim: list[Edge] = outer_rim_edges(tab, top=True) + outer_rim_edges(
        tab, top=False
    )
    try:
        tab = fillet(tab_rim, COLLAR_TAB_FILLET_RADIUS)
    except Exception:
        pass

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
