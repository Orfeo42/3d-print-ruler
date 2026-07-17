from build123d import Cylinder, Pos

from .constants import (
    CONNECTOR_COLLAR_THICKNESS,
    RIVET_HEAD_AIR_GAP,
    RIVET_HEAD_OVERHANG,
    RIVET_HEAD_THICKNESS,
    RIVET_PIN_DIAMETER,
    SEGMENT_CONNECTOR_AIR_GAP,
)
from .geometry import ALIGN_CENTER_BOTTOM, PartLike, require_solid


def build_rivet(hole_radius: float) -> PartLike:
    """Segment-side pin: shaft through the hole, then a head too wide to pull back out."""
    pin_radius: float = RIVET_PIN_DIAMETER / 2
    head_radius: float = hole_radius + RIVET_HEAD_OVERHANG
    shaft_bottom: float = -(
        SEGMENT_CONNECTOR_AIR_GAP + CONNECTOR_COLLAR_THICKNESS + RIVET_HEAD_AIR_GAP
    )
    head_bottom: float = shaft_bottom - RIVET_HEAD_THICKNESS

    shaft: PartLike = Pos(0, 0, shaft_bottom) * Cylinder(
        radius=pin_radius, height=0.5 - shaft_bottom, align=ALIGN_CENTER_BOTTOM
    )
    head: PartLike = Pos(0, 0, head_bottom) * Cylinder(
        radius=head_radius, height=RIVET_HEAD_THICKNESS, align=ALIGN_CENTER_BOTTOM
    )
    return require_solid(shaft + head)
