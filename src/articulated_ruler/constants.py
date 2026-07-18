from typing import Final

SEGMENT_LENGTH: Final[float] = 25.4
SEGMENT_THICKNESS: Final[float] = 3.2

RIVET_PIN_DIAMETER: Final[float] = 3.4
RIVET_PIN_CLEARANCE: Final[float] = 0.5

CONNECTOR_COLLAR_THICKNESS: Final[float] = 2.2
SEGMENT_CONNECTOR_AIR_GAP: Final[float] = 0.15
RIVET_HEAD_AIR_GAP: Final[float] = 0.3
RIVET_HEAD_THICKNESS: Final[float] = 0.8
RIVET_HEAD_OVERHANG: Final[float] = 0.6
RIVET_POCKET_CLEARANCE: Final[float] = 0.4
RIVET_HEAD_RECESS_DEPTH: Final[float] = 0.15

# Must clear a boss ring (COLLAR_OUTER_RADIUS) plus a nozzle-safe bead plus
# clearance margins on both sides of the rivet pocket - 8mm left under 1mm
# total radial room for all three at once.
PIVOT_HOLE_SPACING: Final[float] = 12.0
CONNECTOR_WIDTH: Final[float] = 9.0

CONNECTOR_STACK_DEPTH: Final[float] = -(
    SEGMENT_CONNECTOR_AIR_GAP
    + CONNECTOR_COLLAR_THICKNESS
    + RIVET_HEAD_AIR_GAP
    + RIVET_HEAD_THICKNESS
    + RIVET_HEAD_RECESS_DEPTH
)

THICKENED_MIDDLE_CLEARANCE: Final[float] = CONNECTOR_WIDTH / 2 + 0.3

TIP_REST_GAP: Final[float] = 0.2

# The tip cap is concentric with its own pivot; sizing the inset to exactly
# this radius lands the cap flush on the Segment's own edge with no
# overhang, and two neighbouring caps end up exactly TIP_REST_GAP apart at
# every fold angle (two fixed discs, radius + radius + gap = PIVOT_HOLE_SPACING).
TIP_CAP_RADIUS: Final[float] = (PIVOT_HOLE_SPACING - TIP_REST_GAP) / 2
SEGMENT_WIDTH: Final[float] = 2 * TIP_CAP_RADIUS

# A free tip's cap fillet radius is exactly half its own box width - an exact
# tangency OCCT refuses outright regardless of size. Shave a numerically
# safe sliver off, well below FDM resolution.
TANGENT_FILLET_EPSILON: Final[float] = 0.02
FREE_TIP_FILLET_RADIUS: Final[float] = TIP_CAP_RADIUS - TANGENT_FILLET_EPSILON

TOP_EDGE_CHAMFER: Final[float] = 0.4
BOTTOM_EDGE_FILLET_RADIUS: Final[float] = TOP_EDGE_CHAMFER
MIDDLE_STEP_FILLET_RADIUS: Final[float] = 1.2

# How far one Segment rotates, relative to its fixed neighbour+Connector,
# before pinching shut - measured empirically at ~93 degrees on this
# geometry (ad hoc collision sweep, see DEVELOPMENT.md). A few degrees of
# margin short of that measured limit.
FOLD_CLOSED_ANGLE_DEG: Final[float] = 90.0

RIVET_POCKET_RADIUS: Final[float] = (
    (RIVET_PIN_DIAMETER + RIVET_PIN_CLEARANCE) / 2
    + RIVET_HEAD_OVERHANG
    + RIVET_POCKET_CLEARANCE
)

COLLAR_INNER_RADIUS: Final[float] = RIVET_POCKET_RADIUS + 0.15
COLLAR_HEIGHT: Final[float] = 1.2
COLLAR_Z_CLEARANCE: Final[float] = 0.2
COLLAR_RADIAL_CLEARANCE: Final[float] = 0.15

# The whole radius protrudes: the bead's own centre sits on the boss wall.
# 0.65 (up from 0.5) for a stronger, more resolvable catch.
COLLAR_BEAD_RADIUS: Final[float] = 0.65
COLLAR_GROOVE_RADIUS: Final[float] = COLLAR_BEAD_RADIUS + 0.1
COLLAR_BEAD_FILLET_RADIUS: Final[float] = 0.15

# 0.15 fails outright on the tab's sharp apex vertex; 0.08-0.12 "succeed" as
# a boolean op but mesh into 40x the face count (near-degenerate slivers)
# and break watertightness once combined with the rest of the assembly.
COLLAR_TAB_FILLET_RADIUS: Final[float] = 0.05

# Stays inside the Connector's own cap edge even with the bead at its
# biggest reach, plus a safety margin.
COLLAR_OUTER_RADIUS: Final[float] = CONNECTOR_WIDTH / 2 - COLLAR_BEAD_RADIUS - 0.2

COLLAR_TAB_HALF_ANGLE_DEG: Final[float] = 36.0
COLLAR_TAB_RADIAL_THICKNESS: Final[float] = 1.0

# True zero clearance fuses the Segment and Connector into one solid at the
# tab (verified: `seg + conn` came back as 1 solid with genuine nonzero
# intersection volume), welding the joint shut instead of just making it
# stiff. This is a bare, near-zero but REAL gap.
COLLAR_TAB_RADIAL_CLEARANCE: Final[float] = 0.05

# Tab grows straight off the Segment's own underside (the recess opening),
# no air gap below it. The old 0.1 lift existed because an EARLIER tab
# shape (the thin cantilever) failed the Segment's 0.4mm bottom-rim fillet
# when its own thin edges sat exactly at z=0 - verified the current
# hanging-band shape builds clean at z=0, so the lift is gone.
COLLAR_TAB_Z_OFFSET: Final[float] = 0.0

# How far the tab's own peak pokes past RECESS_OUTER_RADIUS into solid
# Segment material - its own anchor, no separate root piece.
COLLAR_TAB_ANCHOR_OVERREACH: Final[float] = 0.15

RECESS_INNER_RADIUS: Final[float] = COLLAR_INNER_RADIUS - COLLAR_RADIAL_CLEARANCE
RECESS_OUTER_RADIUS: Final[float] = (
    COLLAR_OUTER_RADIUS
    + COLLAR_TAB_RADIAL_CLEARANCE
    + COLLAR_TAB_RADIAL_THICKNESS
    + COLLAR_RADIAL_CLEARANCE
)
RECESS_HEIGHT: Final[float] = (
    -SEGMENT_CONNECTOR_AIR_GAP + COLLAR_HEIGHT + COLLAR_Z_CLEARANCE
)

# Full channel height: the tab runs as one piece from the Segment's own
# underside (z=0, the recess opening) all the way up to the recess ceiling,
# fusing with the Segment there too - not a short band floating mid-channel.
COLLAR_TAB_HEIGHT: Final[float] = RECESS_HEIGHT

NUM_SEGMENTS: Final[int] = 3
