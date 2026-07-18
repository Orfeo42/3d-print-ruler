if __name__ == "__main__" and __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "articulated_ruler"

from collections.abc import Sequence
from dataclasses import dataclass

from build123d import Align, Axis, Box, Cylinder, Edge, Part, Pos, Rotation, chamfer, fillet

from .constants import (
    CONNECTOR_STACK_DEPTH,
    FREE_TIP_FILLET_RADIUS,
    MIDDLE_STEP_FILLET_RADIUS,
    NUM_SEGMENTS,
    RIVET_PIN_CLEARANCE,
    RIVET_PIN_DIAMETER,
    SEGMENT_LENGTH,
    SEGMENT_THICKNESS,
    SEGMENT_WIDTH,
    THICKENED_MIDDLE_CLEARANCE,
    TIP_CAP_RADIUS,
    TIP_REST_GAP,
    TOP_EDGE_CHAMFER,
    BOTTOM_EDGE_FILLET_RADIUS,
)
from .geometry import ALIGN_CENTER_BOTTOM, ALIGN_LEFT_BOTTOM, Align3, PartLike, outer_rim_edges, require_solid
from .pivot_collar import build_recess, build_tab, build_tab_relief
from .rivet import build_rivet


@dataclass(frozen=True, slots=True)
class SegmentSpec:
    """`width` must stay >= 2 * TIP_CAP_RADIUS (fixed globally by
    PIVOT_HOLE_SPACING/TIP_REST_GAP) or a jointed tip's round cap has
    nowhere to fit."""

    length: float = SEGMENT_LENGTH
    width: float = SEGMENT_WIDTH
    thickness: float = SEGMENT_THICKNESS


def default_specs(num_segments: int = NUM_SEGMENTS) -> list[SegmentSpec]:
    return [SegmentSpec() for _ in range(num_segments)]


def total_length(specs: Sequence[SegmentSpec]) -> float:
    return sum(spec.length for spec in specs) + (len(specs) - 1) * TIP_REST_GAP


def _jointed_tip_bottom_rim_edges(
    part: PartLike, spec: SegmentSpec, *, left_joint: bool, right_joint: bool
) -> list[Edge]:
    """A jointed tip's own round nose, at the Segment's own bottom plane
    (z=0) - fused with no dividing edge to the plain box area behind it, so
    a bbox-filtered face search misses it; instead take that face's
    outer_wire and keep only the edges within the tip's own cap radius."""
    edges: list[Edge] = []
    for jointed, in_tip_zone in (
        (left_joint, lambda bb: bb.max.X < TIP_CAP_RADIUS + 1e-6),
        (right_joint, lambda bb: bb.min.X > spec.length - TIP_CAP_RADIUS - 1e-6),
    ):
        if not jointed:
            continue
        for face in part.faces():
            bbox = face.bounding_box()
            if bbox.max.Z - bbox.min.Z > 1e-6 or abs(bbox.min.Z) > 1e-6:
                continue
            edges += [e for e in face.outer_wire().edges() if in_tip_zone(e.bounding_box())]
    return edges


def build_segment(
    spec: SegmentSpec, *, left_joint: bool = True, right_joint: bool = True
) -> PartLike:
    hole_radius: float = (RIVET_PIN_DIAMETER + RIVET_PIN_CLEARANCE) / 2
    box: Part = Box(spec.length, spec.width, spec.thickness, align=ALIGN_LEFT_BOTTOM)

    free_edges: list[Edge] = []
    if not left_joint:
        free_edges += [
            e for e in box.edges().filter_by(Axis.Z) if e.bounding_box().min.X < spec.length / 2
        ]
    if not right_joint:
        free_edges += [
            e for e in box.edges().filter_by(Axis.Z) if e.bounding_box().min.X > spec.length / 2
        ]
    part: PartLike = fillet(free_edges, radius=FREE_TIP_FILLET_RADIUS) if free_edges else box

    # The tab (and its relief void) point their socket toward the Segment's
    # interior on both tips - the right tip's is rotated 180deg. Pointing it
    # at the tip's own nose would leave only a ~0.4mm wall between the
    # relief void and the cap edge, a sliver the bottom rim fillet then
    # eats through. The catch itself is mirror symmetric, so orientation is
    # free.
    for pivot_x, region_lo, tab_rotation_deg, jointed in (
        (TIP_CAP_RADIUS, 0.0, 0.0, left_joint),
        (spec.length - TIP_CAP_RADIUS, spec.length - TIP_CAP_RADIUS, 180.0, right_joint),
    ):
        if not jointed:
            continue
        region: PartLike = Pos(region_lo, 0, 0) * Box(
            TIP_CAP_RADIUS, spec.width, spec.thickness, align=ALIGN_LEFT_BOTTOM
        )
        roundoff: PartLike = Pos(pivot_x, 0, 0) * Cylinder(
            radius=TIP_CAP_RADIUS, height=spec.thickness, align=ALIGN_CENTER_BOTTOM
        )
        corners: PartLike = region - roundoff
        part = require_solid(part - corners + Pos(pivot_x, 0, 0) * build_rivet(hole_radius))
        orient = Pos(pivot_x, 0, 0) * Rotation(0, 0, tab_rotation_deg)
        part = require_solid(
            part
            - Pos(pivot_x, 0, 0) * build_recess()
            - orient * build_tab_relief()
            + orient * build_tab()
        )

    mid_lo: float = (TIP_CAP_RADIUS + THICKENED_MIDDLE_CLEARANCE) if left_joint else 0.0
    mid_hi: float = (
        (spec.length - TIP_CAP_RADIUS - THICKENED_MIDDLE_CLEARANCE) if right_joint else spec.length
    )
    if mid_hi > mid_lo:
        thick_height: float = -CONNECTOR_STACK_DEPTH
        thick_align: Align3 = (Align.MIN, Align.CENTER, Align.MAX)
        thick_box: Part = Box(mid_hi - mid_lo, spec.width, thick_height, align=thick_align)

        far_x: float = mid_hi - mid_lo
        tip_edges: list[Edge] = []
        if not left_joint:
            tip_edges += [
                e for e in thick_box.edges().filter_by(Axis.Z) if e.bounding_box().min.X < 1e-6
            ]
        if not right_joint:
            tip_edges += [
                e
                for e in thick_box.edges().filter_by(Axis.Z)
                if e.bounding_box().min.X > far_x - 1e-6
            ]
        thick: PartLike = (
            fillet(tip_edges, radius=FREE_TIP_FILLET_RADIUS) if tip_edges else thick_box
        )

        step_z: float = -thick_height
        step_edges: list[Edge] = []
        if left_joint:
            step_edges += [
                e
                for e in thick.edges().filter_by(Axis.Y)
                if e.bounding_box().min.X < 1e-6 and abs(e.bounding_box().min.Z - step_z) < 1e-6
            ]
        if right_joint:
            step_edges += [
                e
                for e in thick.edges().filter_by(Axis.Y)
                if e.bounding_box().min.X > far_x - 1e-6
                and abs(e.bounding_box().min.Z - step_z) < 1e-6
            ]
        step_radius: float = min(MIDDLE_STEP_FILLET_RADIUS, thick_height * 0.9, far_x * 0.9)
        if step_edges and step_radius > 0:
            thick = fillet(step_edges, radius=step_radius)

        thick = Pos(mid_lo, 0, 0) * thick
        part = require_solid(part + thick)

    top: PartLike = require_solid(chamfer(outer_rim_edges(part, top=True), TOP_EDGE_CHAMFER))
    bottom_edges: list[Edge] = outer_rim_edges(top, top=False) + _jointed_tip_bottom_rim_edges(
        top, spec, left_joint=left_joint, right_joint=right_joint
    )
    return require_solid(fillet(bottom_edges, BOTTOM_EDGE_FILLET_RADIUS))


if __name__ == "__main__":
    part = build_segment(SegmentSpec())
    try:
        from ocp_vscode import show

        show(part)
    except Exception as exc:
        print(f"preview skipped ({type(exc).__name__}: {exc})")
