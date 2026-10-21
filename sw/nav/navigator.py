"""Track where the AGV is on the map and which way it faces.

State holds three things: the junction the AGV is at, the compass direction
it faces there, and - for a route typed out in advance - how far along that
list it has got.

The model the rest of the navigation depends on:

    current_node        the junction the AGV has most recently reached
    current_direction   the compass direction it faces at that junction

A junction event means "I have arrived at the node in front of me", so the
state is advanced with arrive(). A turn changes which way the AGV faces but
not where it is, so it is applied with face(). Keeping the two separate is
what lets the AGV be told to turn at a junction it is standing on, which is
how every leg begins.

State does not watch the sensors. Nothing here knows whether the AGV really
completed a turn; it records what the AGV was told to do.
"""

from .map import JUNCTIONS, ahead, find_target_direction

# Clockwise compass order lets us calculate the relative turn with one
# modular subtraction: 0=straight, 1=right, 2=U-turn, 3=left.
_COMPASS_ORDER = ("N", "E", "S", "W")
_TURN_BY_OFFSET = {
    0: "straight",
    1: "right",
    2: "uturn",
    3: "left",
}

STRAIGHT = "straight"
LEFT = "left"
RIGHT = "right"
UTURN = "uturn"


class State:
    """Track the planned route position and the AGV's compass heading.

    `current_node` is the route junction the AGV is considered to be at.
    `current_direction` is the compass direction it faces there.
    `sequence_index` identifies that node in the route sequence.

    `sequence` is optional. It is the explicit route M1 ran, kept so that a
    typed-out route can still be driven; the mission planner sets no
    sequence at all and steers by target instead.
    """
    def __init__(self, initial_node, initial_direction, sequence=None):
        if initial_node not in JUNCTIONS:
            raise ValueError("Unknown initial node: {}".format(initial_node))
        self.current_node = initial_node

        # Normalize case, then require the same single-letter directions used
        # by the map so an invalid heading is caught at setup time.
        direction = initial_direction.upper()
        if direction not in _COMPASS_ORDER:
            raise ValueError(
                "Unknown initial direction: {}".format(initial_direction))
        self.current_direction = direction

        if sequence is None:
            self.sequence = ()
            self.sequence_index = 0
        else:
            self.follow(sequence)

    # ------------------------------------------------------------ primitives

    def _turn_and_heading(self, node):
        """(relative turn, new compass heading) for leaving here for `node`."""
        # The map gives the compass direction of the exit leading to the
        # target node; convert that absolute direction into a relative turn.
        target_direction = find_target_direction(self.current_node, node)

        current_index = _COMPASS_ORDER.index(self.current_direction)
        target_index = _COMPASS_ORDER.index(target_direction)
        offset = (target_index - current_index) % len(_COMPASS_ORDER)
        return _TURN_BY_OFFSET[offset], target_direction

    def turn_to(self, node):
        """The turn needed to leave here for `node`, changing nothing.

        Also answers "which side of the AGV is that bay on?", because the
        turn into a bay and the side it sits on are the same thing.
        """
        return self._turn_and_heading(node)[0]

    def face(self, node):
        """Turn on the spot to face `node`. Returns the turn made.

        The AGV stays where it is: a turn at a junction does not move it to
        the next one, and pretending otherwise is what puts a route one
        junction out of step.
        """
        turn, heading = self._turn_and_heading(node)
        self.current_direction = heading
        return turn

    def arrive(self, node):
        """Record reaching `node`, which must be the junction being faced."""
        expected = ahead(self.current_node, self.current_direction)
        if node != expected:
            raise ValueError(
                "Cannot arrive at {} from {} facing {}: that way leads to "
                "{}".format(node, self.current_node, self.current_direction,
                            expected))
        self.current_node = node

    def reverse_to(self, node):
        """Record reversing to `node`, which must be directly behind.

        Backing out of a bay is not a route step the map describes - the AGV
        ends up at the corridor junction while still facing into the bay it
        has left - so it is recorded as its own kind of move. The heading
        does not change, because reversing does not turn the AGV round.
        """
        behind = _COMPASS_ORDER[
            (_COMPASS_ORDER.index(self.current_direction) + 2) % 4]
        expected = ahead(self.current_node, behind)
        if node != expected:
            raise ValueError(
                "Cannot reverse to {} from {} facing {}: behind lies "
                "{}".format(node, self.current_node, self.current_direction,
                            expected))
        self.current_node = node

    def move_to(self, node):
        """Face `node` and arrive there in one step. Returns the turn made."""
        turn = self.face(node)
        self.arrive(node)
        return turn

    # --------------------------------------------------------- typed routes

    def follow(self, nodes):
        """Replace the planned route with `nodes`, starting from here."""
        sequence = tuple(nodes)

        # The first route entry represents the AGV's starting junction.
        if not sequence or sequence[0] != self.current_node:
            raise ValueError(
                "Route must start at the current node {}".format(
                    self.current_node))

        # Check every planned step once, rather than allowing an impossible
        # map transition to fail after the AGV has started.
        for index in range(1, len(sequence)):
            previous_node = sequence[index - 1]
            next_node = sequence[index]
            try:
                find_target_direction(previous_node, next_node)
            except (KeyError, StopIteration):
                raise ValueError(
                    "No mapped connection from {} to {}".format(
                        previous_node, next_node))

        self.sequence = sequence
        self.sequence_index = 0

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

        turn = self.move_to(target_node)
        self.sequence_index += 1
        return turn


# Keep the descriptive name available for callers that prefer it.
Navigator = State
