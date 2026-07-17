"""Print-in-place articulated ruler: Segment - Connector - Segment - ... - Segment.

See DEVELOPMENT.md for design history and rationale.
"""

if __name__ == "__main__" and __package__ in (None, ""):
    import sys
    from pathlib import Path as _Path

    sys.path.insert(0, str(_Path(__file__).resolve().parents[1]))
    __package__ = "articulated_ruler"

from collections.abc import Sequence
from pathlib import Path

from build123d import Pos, export_step, export_stl

from .connector import build_connector
from .constants import TIP_REST_GAP
from .geometry import PartLike, require_solid
from .segment import SegmentSpec, build_segment, default_specs, total_length


def _segment_piece(
    spec: SegmentSpec, x: float, *, left_joint: bool, right_joint: bool
) -> PartLike:
    return Pos(x, 0, 0) * build_segment(spec, left_joint=left_joint, right_joint=right_joint)


def _junction_piece(x: float) -> PartLike:
    return Pos(x, 0, 0) * build_connector()


def build_ruler(specs: Sequence[SegmentSpec] | None = None) -> PartLike:
    """One Segment per entry in `specs` (default: `default_specs()`), a
    Connector at every seam. Each Segment keeps exactly its own
    `spec.length` - the chain's two free ends are simply unjointed, not
    shortened or lengthened. A running cursor advances by each Segment's own
    length plus TIP_REST_GAP, so Segments of different lengths still line up."""
    if specs is None:
        specs = default_specs()

    pieces: list[PartLike] = []
    cursor: float = 0.0
    for i, spec in enumerate(specs):
        left_joint: bool = i > 0
        right_joint: bool = i < len(specs) - 1
        pieces.append(_segment_piece(spec, cursor, left_joint=left_joint, right_joint=right_joint))
        seg_end: float = cursor + spec.length
        if right_joint:
            pieces.append(_junction_piece(seg_end + TIP_REST_GAP / 2))
            cursor = seg_end + TIP_REST_GAP

    chain: PartLike = pieces[0]
    for piece in pieces[1:]:
        chain = require_solid(chain + piece)
    return chain


def export_all(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    specs: list[SegmentSpec] = default_specs()
    part: PartLike = build_ruler(specs)
    stl_path: Path = out_dir / "ruler.stl"
    step_path: Path = out_dir / "ruler.step"
    export_stl(part, str(stl_path))
    export_step(part, str(step_path))
    print(f"wrote {stl_path}")
    print(f"wrote {step_path}")
    print(f"extended length: {total_length(specs):.1f}mm ({len(specs)} segments)")


def preview() -> None:
    """Show the ruler in the VSCode OCP CAD Viewer (open, port 3939)."""
    from ocp_vscode import show

    show(build_ruler())


if __name__ == "__main__":
    try:
        preview()
    except Exception as exc:
        print(f"preview skipped ({type(exc).__name__}: {exc})")
    export_all(Path(__file__).resolve().parents[2] / "output")
