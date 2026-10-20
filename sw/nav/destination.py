"""Work out where a package has to go.

The task spec fixes the whole rule, and no QR code is needed for any of it:

    "Boxes collected from the left-hand areas (when looking at the traffic
     lights), go to 'A' rack, those on the right-hand area to 'B' rack. The
     boxes with north pole magnets need to be delivered to the top racks,
     those with south pole magnets to the bottom racks"

So two independent facts decide the destination:

    rack    A or B      from which side the package was collected
    level   top/bottom  from the magnet pole

Only the second needs a sensor. The AGV already knows which loading bay it
drove to, so the rack letter costs nothing to determine - it is route
knowledge, not a measurement.

The bay number is not decided here. The spec says to "only deliver if the
location is not already occupied", so the delivery step walks the six
candidate bays in order and takes the first free one.
"""

from drivers.magnet import NORTH, SOUTH

RACK_A = "A"
RACK_B = "B"

TOP = "top"
BOTTOM = "bottom"

LEFT = "left"
RIGHT = "right"

BAYS_PER_RACK = 6

# How (rack, level) maps onto the junction names in map.py.
#
# CHECK this against the table before the competition. The reasoning:
#
#   map.py has four runs of six bays - RU1-6, LU1-6, RB1-6, LB1-6 - which is
#   24, exactly the number of storage locations. It also has R1_BAY, R2_BAY,
#   L1_BAY and L2_BAY, which is 4, exactly the number of loading bays the
#   spec places packages on at the start.
#
#   In the names, R and L are the side when facing up the table and U and B
#   are the upper and bottom decks. The U corridor is reached from HOME by
#   going north through UU, which is the ramp to the top deck.
#
#   The spec puts the left-hand collection areas on rack A, so A is the L
#   side and B is the R side.
#
# If the team's naming means something else, this is the one table to fix.
_BAY_PREFIX = {
    (RACK_A, TOP): "LU",
    (RACK_A, BOTTOM): "LB",
    (RACK_B, TOP): "RU",
    (RACK_B, BOTTOM): "RB",
}

# The loading bays packages are collected from, and which rack each feeds.
LOADING_BAYS = {
    "L1_BAY": LEFT,
    "L2_BAY": LEFT,
    "R1_BAY": RIGHT,
    "R2_BAY": RIGHT,
}


def rack_for(pickup_side):
    """LEFT collects to rack A, RIGHT to rack B."""
    if pickup_side == LEFT:
        return RACK_A
    if pickup_side == RIGHT:
        return RACK_B
    raise ValueError("Unknown pickup side: {}".format(pickup_side))


def level_for(pole):
    """North goes to the top racks, south to the bottom ones."""
    if pole == NORTH:
        return TOP
    if pole == SOUTH:
        return BOTTOM
    raise ValueError("Unknown pole: {}".format(pole))


def side_of_loading_bay(node):
    """Which side a loading bay node sits on, for rack_for()."""
    try:
        return LOADING_BAYS[node]
    except KeyError:
        raise ValueError("Not a loading bay: {}".format(node))


def destination(pole, pickup_side):
    """Return (rack, level) for a package.

    pickup_side is LEFT or RIGHT; side_of_loading_bay() derives it from the
    node the package was collected at.
    """
    return rack_for(pickup_side), level_for(pole)


def bay_nodes(rack, level):
    """The six candidate bay nodes for a destination, in visiting order.

    Delivery tries these in turn and takes the first unoccupied one, which is
    what the spec means by "only deliver if the location is not already
    occupied".
    """
    try:
        prefix = _BAY_PREFIX[(rack, level)]
    except KeyError:
        raise ValueError("Unknown destination: {} {}".format(rack, level))
    return tuple("{}{}_BAY".format(prefix, n)
                 for n in range(1, BAYS_PER_RACK + 1))
