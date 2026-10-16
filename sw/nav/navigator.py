from map import find_target_direction


# Clockwise compass order lets us calculate the relative turn with one
# modular subtraction: 0=straight, 1=right, 2=U-turn, 3=left.
_COMPASS_ORDER = ("N", "E", "S", "W")
_TURN_BY_OFFSET = {
    0: "straight",
    1: "right",
    2: "uturn",
    3: "left",
}


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

        # Check every planned step once at construction, rather than allowing
        # an impossible map transition to fail after the AGV has started.
        for index in range(1, len(self.sequence)):
            previous_node = self.sequence[index - 1]
            next_node = self.sequence[index]
            try:
                find_target_direction(previous_node, next_node)
            except (KeyError, StopIteration):
                raise ValueError(
                    "No mapped connection from {} to {}".format(
                        previous_node, next_node))

    def next_node(self):
        """Return the next planned junction, or None when the route is done."""
        next_index = self.sequence_index + 1
        if next_index < len(self.sequence):
            return self.sequence[next_index]
        return None

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
    
