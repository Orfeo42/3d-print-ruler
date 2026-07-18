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


_TAB_NARROW_HALF_ANGLE_DEG: float = 18.0
_TAB_TAPER_SEGMENTS: int = 16
_TAB_FAR_ARC_SEGMENTS: int = 16


def build_tab() -> PartLike:
    """Segment-side catch living inside the recess void, centred on the
    pivot: narrow at the hole end (flush against the boss's own wall, where
    the groove matches the bead's own fixed position), smoothly flaring out
    to a wide, round-fronted bulge at the far end, fusing with solid,
    un-recessed Segment material past the recess's own outer radius."""
    narrow_half_angle: float = math.radians(_TAB_NARROW_HALF_ANGLE_DEG)
    wide_half_angle: float = math.radians(COLLAR_TAB_HALF_ANGLE_DEG)
    tab_inner_radius: float = COLLAR_OUTER_RADIUS + COLLAR_TAB_RADIAL_CLEARANCE
    tab_outer_radius: float = RECESS_OUTER_RADIUS + COLLAR_TAB_ANCHOR_OVERREACH

    near_side: list[tuple[float, float]] = []
    for i in range(_TAB_TAPER_SEGMENTS + 1):
        # Linear in t, not eased - an eased curve bunches points up near
        # both ends, producing near-zero-length edges there that no fillet
        # radius (down to 0.001mm) can round.
        t: float = i / _TAB_TAPER_SEGMENTS
        angle: float = -(narrow_half_angle + (wide_half_angle - narrow_half_angle) * t)
        radius: float = tab_inner_radius + (tab_outer_radius - tab_inner_radius) * t
        near_side.append((radius * math.cos(angle), radius * math.sin(angle)))

    far_arc: list[tuple[float, float]] = [
        (
            tab_outer_radius * math.cos(-wide_half_angle + 2 * wide_half_angle * i / _TAB_FAR_ARC_SEGMENTS),
            tab_outer_radius * math.sin(-wide_half_angle + 2 * wide_half_angle * i / _TAB_FAR_ARC_SEGMENTS),
        )
        for i in range(1, _TAB_FAR_ARC_SEGMENTS)
    ]
    far_side: list[tuple[float, float]] = [(x, -y) for x, y in reversed(near_side)]
    # Closes the loop back to near_side's start at constant tab_inner_radius
    # - an explicit arc, not the polygon's own implicit straight closing
    # edge, which would cut a chord inside the boss's own outer wall and
    # collide with it.
    near_arc: list[tuple[float, float]] = [
        (
            tab_inner_radius * math.cos(narrow_half_angle - 2 * narrow_half_angle * i / _TAB_FAR_ARC_SEGMENTS),
            tab_inner_radius * math.sin(narrow_half_angle - 2 * narrow_half_angle * i / _TAB_FAR_ARC_SEGMENTS),
        )
        for i in range(1, _TAB_FAR_ARC_SEGMENTS)
    ]

    sketch = Polygon(*near_side, *far_arc, *far_side, *near_arc)
    # This point winding extrudes towards +Z with a POSITIVE amount (the
    # opposite sign from the old single-peak polygon) - verified directly:
    # amount=-HEIGHT put the tab at [-HEIGHT, 0], entirely missing the
    # groove cylinder at [Z_OFFSET, Z_OFFSET+HEIGHT], so the subtraction
    # silently removed nothing.
    tab: PartLike = Pos(0, 0, COLLAR_TAB_Z_OFFSET) * require_solid(extrude(sketch, amount=COLLAR_TAB_HEIGHT))

    tab_rim: list[Edge] = outer_rim_edges(tab, top=True) + outer_rim_edges(tab, top=False)
    try:
        tab = fillet(tab_rim, COLLAR_TAB_FILLET_RADIUS)
    except Exception:
        pass

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
