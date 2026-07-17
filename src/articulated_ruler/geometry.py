from typing import Final

from build123d import Align, Axis, Compound, Curve, Edge, Part, Wire, chamfer

type Align3 = tuple[Align, Align, Align]
ALIGN_LEFT_BOTTOM: Final[Align3] = (Align.MIN, Align.CENTER, Align.MIN)
ALIGN_CENTER_BOTTOM: Final[Align3] = (Align.CENTER, Align.CENTER, Align.MIN)
ALIGN_CENTER_TOP: Final[Align3] = (Align.CENTER, Align.CENTER, Align.MAX)

type PartLike = Part | Compound | Wire


def require_solid(shape: Part | Compound | Wire | Edge | Curve) -> PartLike:
    if isinstance(shape, (Part, Compound, Wire)):
        return shape
    raise TypeError(f"expected a solid Part/Compound/Wire, got {type(shape).__name__}")


def outer_rim_edges(part: PartLike, *, top: bool) -> list[Edge]:
    """Outer-silhouette edges of a solid's Z-extreme face (top or bottom),
    via that face's own `outer_wire()` - not a bare `filter_by(Axis.Z)`,
    which also catches internal horizontal shelf faces (e.g. a stepped
    bore's own step) and leaves inner hole loops untouched."""
    faces_by_z = part.faces().sort_by(Axis.Z)
    face = faces_by_z[-1] if top else faces_by_z[0]
    return face.outer_wire().edges()


def chamfer_horizontal_rims(part: PartLike, radius: float) -> PartLike:
    rim_edges: list[Edge] = outer_rim_edges(part, top=True) + outer_rim_edges(part, top=False)
    return require_solid(chamfer(rim_edges, radius))
