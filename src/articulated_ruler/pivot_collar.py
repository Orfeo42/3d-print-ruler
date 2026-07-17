if __name__ == "__main__" and __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "articulated_ruler"

import math

from build123d import Cylinder, Edge, GeomType, Polygon, Pos, extrude, fillet

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
    COLLAR_TAB_RADIAL_CLEARANCE,
    COLLAR_TAB_Z_OFFSET,
    RECESS_HEIGHT,
    RECESS_INNER_RADIUS,
    RECESS_OUTER_RADIUS,
)
from .geometry import ALIGN_CENTER_BOTTOM, PartLike, outer_rim_edges, require_solid

_TAB_ARC_SEGMENTS: int = 12


def build_recess() -> PartLike:
    """Segment-side void for the pivot collar - a full-circle annulus,
    centred on the local origin (the pivot). Full-circle, not limited to the
    fold's own angular range: the Connector's boss is a fixed full ring in
    the world frame, so any un-recessed Segment material at this radius
    band would rotate straight into it after only a few degrees of fold."""
    return require_solid(
        Cylinder(radius=RECESS_OUTER_RADIUS, height=RECESS_HEIGHT, align=ALIGN_CENTER_BOTTOM)
        - Cylinder(radius=RECESS_INNER_RADIUS, height=RECESS_HEIGHT, align=ALIGN_CENTER_BOTTOM)
    )


def build_tab() -> PartLike:
    """Segment-side catch living inside the recess void, centred on the
    pivot: a solid, rigid V-shaped wedge, flush against the boss's own wall
    (COLLAR_TAB_RADIAL_CLEARANCE is a bare, near-zero real gap) at its two
    angular ends, rising in a straight line to a peak that reaches past the
    recess's own outer radius and fuses with solid, un-recessed Segment
    material there - its own peak is its anchor. The round groove is cut
    through that peak."""
    half_angle: float = math.radians(COLLAR_TAB_HALF_ANGLE_DEG)
    apex_radius: float = RECESS_OUTER_RADIUS + COLLAR_TAB_ANCHOR_OVERREACH
    tab_inner_radius: float = COLLAR_OUTER_RADIUS + COLLAR_TAB_RADIAL_CLEARANCE
    arc_points: list[tuple[float, float]] = [
        (
            tab_inner_radius * math.cos(-half_angle + 2 * half_angle * i / _TAB_ARC_SEGMENTS),
            tab_inner_radius * math.sin(-half_angle + 2 * half_angle * i / _TAB_ARC_SEGMENTS),
        )
        for i in range(_TAB_ARC_SEGMENTS + 1)
    ]
    sketch = Polygon(*arc_points, (apex_radius, 0.0))
    tab: PartLike = Pos(0, 0, COLLAR_TAB_Z_OFFSET) * require_solid(extrude(sketch, amount=-COLLAR_TAB_HEIGHT))

    tab_rim: list[Edge] = outer_rim_edges(tab, top=True) + outer_rim_edges(tab, top=False)
    tab = fillet(tab_rim, COLLAR_TAB_FILLET_RADIUS)

    groove_cylinder: PartLike = Cylinder(
        radius=COLLAR_GROOVE_RADIUS, height=COLLAR_TAB_HEIGHT, align=ALIGN_CENTER_BOTTOM
    )
    groove_cylinder = fillet(
        groove_cylinder.edges().filter_by(GeomType.CIRCLE), radius=COLLAR_BEAD_FILLET_RADIUS
    )
    groove: PartLike = Pos(COLLAR_OUTER_RADIUS, 0, COLLAR_TAB_Z_OFFSET) * groove_cylinder
    return require_solid(tab - groove)


def build_boss(target_angles_deg: tuple[float, float]) -> PartLike:
    """Connector-side raised ring, centred on the local origin (the pivot),
    with a bead protruding from its own wall at each target fold angle -
    see `build_tab` for the matching catch."""
    boss: PartLike = require_solid(
        Cylinder(radius=COLLAR_OUTER_RADIUS, height=COLLAR_HEIGHT, align=ALIGN_CENTER_BOTTOM)
        - Cylinder(radius=COLLAR_INNER_RADIUS, height=COLLAR_HEIGHT, align=ALIGN_CENTER_BOTTOM)
    )
    for target_deg in target_angles_deg:
        rad: float = math.radians(target_deg)
        bead_x: float = COLLAR_OUTER_RADIUS * math.cos(rad)
        bead_y: float = COLLAR_OUTER_RADIUS * math.sin(rad)
        bead_cylinder: PartLike = Cylinder(
            radius=COLLAR_BEAD_RADIUS, height=COLLAR_HEIGHT, align=ALIGN_CENTER_BOTTOM
        )
        bead_cylinder = fillet(
            bead_cylinder.edges().filter_by(GeomType.CIRCLE), radius=COLLAR_BEAD_FILLET_RADIUS
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
