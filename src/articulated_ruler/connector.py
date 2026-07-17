if __name__ == "__main__" and __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "articulated_ruler"

from build123d import Box, Cylinder, Pos

from .constants import (
    CONNECTOR_COLLAR_THICKNESS,
    CONNECTOR_WIDTH,
    FOLD_CLOSED_ANGLE_DEG,
    PIVOT_HOLE_SPACING,
    RIVET_HEAD_AIR_GAP,
    RIVET_HEAD_OVERHANG,
    RIVET_HEAD_RECESS_DEPTH,
    RIVET_HEAD_THICKNESS,
    RIVET_PIN_CLEARANCE,
    RIVET_PIN_DIAMETER,
    RIVET_POCKET_CLEARANCE,
    SEGMENT_CONNECTOR_AIR_GAP,
    TOP_EDGE_CHAMFER,
)
from .geometry import ALIGN_CENTER_TOP, PartLike, chamfer_horizontal_rims, require_solid
from .pivot_collar import build_boss


def build_connector() -> PartLike:
    """Stadium-shaped plate below the Segments. Each end is a stepped bore:
    a narrow collar bore (shaft clearance) on top, then a wide pocket (head
    clearance) that stays open at the Connector's own flat bottom face, with
    the rivet head recessed above that face."""
    hole_radius: float = (RIVET_PIN_DIAMETER + RIVET_PIN_CLEARANCE) / 2
    head_radius: float = hole_radius + RIVET_HEAD_OVERHANG
    pocket_radius: float = head_radius + RIVET_POCKET_CLEARANCE
    cap_radius: float = CONNECTOR_WIDTH / 2

    top: float = -SEGMENT_CONNECTOR_AIR_GAP
    collar_bottom: float = top - CONNECTOR_COLLAR_THICKNESS
    head_bottom: float = collar_bottom - RIVET_HEAD_AIR_GAP - RIVET_HEAD_THICKNESS
    bottom: float = head_bottom - RIVET_HEAD_RECESS_DEPTH
    thickness: float = top - bottom

    part: PartLike = Pos(0, 0, top) * Box(
        PIVOT_HOLE_SPACING, CONNECTOR_WIDTH, thickness, align=ALIGN_CENTER_TOP
    )
    for x in (-PIVOT_HOLE_SPACING / 2, PIVOT_HOLE_SPACING / 2):
        cap: PartLike = Pos(x, 0, top) * Cylinder(radius=cap_radius, height=thickness, align=ALIGN_CENTER_TOP)
        part = require_solid(part + cap)
    for x in (-PIVOT_HOLE_SPACING / 2, PIVOT_HOLE_SPACING / 2):
        collar_bore: PartLike = Pos(x, 0, top) * Cylinder(
            radius=hole_radius, height=top - collar_bottom, align=ALIGN_CENTER_TOP
        )
        pocket: PartLike = Pos(x, 0, collar_bottom) * Cylinder(
            radius=pocket_radius, height=collar_bottom - bottom, align=ALIGN_CENTER_TOP
        )
        part = part - collar_bore - pocket
    part = chamfer_horizontal_rims(require_solid(part), TOP_EDGE_CHAMFER)

    # Added after the rim chamfer, not before: it rises above the
    # Connector's own top surface, so `outer_rim_edges` would otherwise pick
    # the boss's own tiny top ring as the Connector's "top face" instead of
    # its real outer silhouette. Not user-facing, so it skips its own rim
    # chamfer too.
    for x in (-PIVOT_HOLE_SPACING / 2, PIVOT_HOLE_SPACING / 2):
        boss: PartLike = Pos(x, 0, top) * build_boss((FOLD_CLOSED_ANGLE_DEG, -FOLD_CLOSED_ANGLE_DEG))
        part = require_solid(part + boss)
    return part


if __name__ == "__main__":
    part = build_connector()
    try:
        from ocp_vscode import show

        show(part)
    except Exception as exc:
        print(f"preview skipped ({type(exc).__name__}: {exc})")
