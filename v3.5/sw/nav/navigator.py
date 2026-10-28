from .map import JUNCTIONS, find_target_direction


# Clockwise compass order lets us calculate the relative turn with one
# modular subtraction: 0=straight, 1=right, 2=U-turn, 3=left.
_COMPASS_ORDER = ("N", "E", "S", "W")
_TURN_BY_OFFSET = {
    0: "straight",
    1: "right",
    2: "uturn",
    3: "left",
}


def _check_route(sequence):
    """Raise if any step of the route is not a mapped connection.

    Checking every step up front stops an impossible map transition failing
    after the AGV has started.
    """
    for index in range(1, len(sequence)):
        previous_node = sequence[index - 1]     # where this step starts
        next_node = sequence[index]             # where this step should lead
        try:
            find_target_direction(previous_node, next_node)
        except (KeyError, StopIteration):
            raise ValueError(
                "No mapped connection from {} to {}".format(
                    previous_node, next_node))


class State:
    """Track the planned route position and the AGV's compass heading.

    `current_node` is the route junction the AGV is considered to be at.
    `current_direction` is the compass direction it faces there.
    `sequence_index` identifies that node in the route sequence.
    """
    def __init__(self, initial_node, initial_direction, sequence):
        self.current_node = initial_node

        # Normalize case, then require the same single-letter directions used
        # by the map so an invalid heading is caught at setup time.
        direction = initial_direction.upper()
        if direction not in _COMPASS_ORDER:
            raise ValueError(
                "Unknown initial direction: {}".format(initial_direction))
        self.current_direction = direction

        # Copy the route so outside changes cannot invalidate the checks below.
        self.sequence = tuple(sequence)
        self.sequence_index = 0

        # The first route entry represents the AGV's starting junction.
        if not self.sequence or self.sequence[0] != initial_node:
            raise ValueError(
                "Route must start at the initial node {}".format(initial_node))
        _check_route(self.sequence)

    def replan(self, nodes):
        """Replace the rest of the route with nodes, which start at current_node.

        The node before current_node is kept, so previous_node() still says
        where the AGV came from - the turn-out helpers in main.py rely on it.
        """
        nodes = tuple(nodes)                    # the new route, from where the AGV is now
        if not nodes or nodes[0] != self.current_node:
            raise ValueError(
                "New route must start at the current node {}".format(self.current_node))
        _check_route(nodes)

        kept = self.sequence[max(0, self.sequence_index - 1):self.sequence_index]  # previous node only, or () at the start
        self.sequence = kept + nodes            # history stays one node long, so it never grows
        self.sequence_index = len(kept)         # points at current_node again

    def next_node(self):
        """Return the next planned junction, or None when the route is done."""
        next_index = self.sequence_index + 1
        if next_index < len(self.sequence):
            return self.sequence[next_index]
        return None

    def previous_node(self):
        """Return the junction visited before current_node, or None at the start."""
        if self.sequence_index > 0:
            return self.sequence[self.sequence_index - 1]
        return None

    def bays_beside(self):
        """Return {"left": bay or None, "right": bay or None} at current_node.

        Uses the current heading, so it must be called before junction_turn()
        advances the state, while current_node is the junction the AGV is
        physically at.
        """
        heading = _COMPASS_ORDER.index(self.current_direction)
        exits = JUNCTIONS[self.current_node]
        beside = {}
        for side, offset in (("left", 3), ("right", 1)):
            node = exits[_COMPASS_ORDER[(heading + offset) % len(_COMPASS_ORDER)]]
            beside[side] = node if node is not None and node.endswith("_BAY") else None
        return beside   # = {"left": None, "right": "example_BAY"}

    def junction_turn(self):
        """Return the relative turn needed for the next route node.

        This method advances the planned state when called. It does not check
        that the AGV has physically completed the turn or reached the node.
        """
        target_node = self.next_node()
        if target_node is None:
            return None

        # The map gives the compass direction of the exit leading to the
        # target node; convert that absolute direction into a relative turn.
        target_direction = find_target_direction(self.current_node, target_node)

        current_index = _COMPASS_ORDER.index(self.current_direction)
        target_index = _COMPASS_ORDER.index(target_direction)
        offset = (target_index - current_index) % len(_COMPASS_ORDER)
        turn = _TURN_BY_OFFSET[offset]

        # Advance the planned position and heading so the next call is based
        # on the next route segment. This is a plan update, not motion feedback.
        self.current_node = target_node
        self.current_direction = target_direction
        self.sequence_index += 1
        return turn


# Keep the descriptive name available for callers that prefer it.
Navigator = State


if __name__ == "__main__":
    state = State(
        initial_node="HOME",
        initial_direction="N",
        sequence=["HOME", "LH"],
    )
    print("Next turn:", state.junction_turn())
    print(state.current_node)
    print(state.current_direction)
    
