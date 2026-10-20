JUNCTIONS = {
    "HOME": {"N": "UU", "E": "RH", "S": None, "W": "LH"},
    "UU": {"N": "U1", "E": None, "S": "HOME", "W": None},
    "U1": {"N": "U2", "E": "RU1_BAY", "S": "UU", "W": "LU1_BAY"},
    "U2": {"N": "U3", "E": "RU2_BAY", "S": "U1", "W": "LU2_BAY"},
    "U3": {"N": "U4", "E": "RU3_BAY", "S": "U2", "W": "LU3_BAY"},
    "U4": {"N": "U5", "E": "RU4_BAY", "S": "U3", "W": "LU4_BAY"},
    "U5": {"N": "U6", "E": "RU5_BAY", "S": "U4", "W": "LU5_BAY"},
    "U6": {"N": None, "E": "RU6_BAY", "S": "U5", "W": "LU6_BAY"},

    "RH": {"N": None, "E": "RC", "S": None, "W": "HOME"},
    "RC": {"N": "R1", "E": None, "S": None, "W": "RH"},
    "R1": {"N": "R2", "E": None, "S": "RC", "W": "R1_BAY"},
    "R2": {"N": "RB1", "E": None, "S": "R1", "W": "R2_BAY"},
    "R1_BAY": {"N": None, "E": "R1", "S": None, "W": None},
    "R2_BAY": {"N": None, "E": "R2", "S": None, "W": None},

    "RB1": {"N": "RB2", "E": None, "S": "R2", "W": "RB1_BAY"},
    "RB2": {"N": "RB3", "E": None, "S": "RB1", "W": "RB2_BAY"},
    "RB3": {"N": "RB4", "E": None, "S": "RB2", "W": "RB3_BAY"},
    "RB4": {"N": "RB5", "E": None, "S": "RB3", "W": "RB4_BAY"},
    "RB5": {"N": "RB6", "E": None, "S": "RB4", "W": "RB5_BAY"},
    "RB6": {"N": None, "E": None, "S": "RB5", "W": "RB6_BAY"},

    "LH": {"N": None, "E": "HOME", "S": None, "W": "LC"},
    "LC": {"N": "L1", "E": "LH", "S": None, "W": None},
    "L1": {"N": "L2", "E": "L1_BAY", "S": "LC", "W": None},
    "L2": {"N": "LB1", "E": "L2_BAY", "S": "L1", "W": None},
    "L1_BAY": {"N": None, "E": None, "S": None, "W": "L1"},
    "L2_BAY": {"N": None, "E": None, "S": None, "W": "L2"},

    "LB1": {"N": "LB2", "E": "LB1_BAY", "S": "L2", "W": None},
    "LB2": {"N": "LB3", "E": "LB2_BAY", "S": "LB1", "W": None},
    "LB3": {"N": "LB4", "E": "LB3_BAY", "S": "LB2", "W": None},
    "LB4": {"N": "LB5", "E": "LB4_BAY", "S": "LB3", "W": None},
    "LB5": {"N": "LB6", "E": "LB5_BAY", "S": "LB4", "W": None},
    "LB6": {"N": None, "E": "LB6_BAY", "S": "LB5", "W": None},

    "RU1_BAY": {"N": None, "E": None, "S": None, "W": "U1"},
    "RU2_BAY": {"N": None, "E": None, "S": None, "W": "U2"},
    "RU3_BAY": {"N": None, "E": None, "S": None, "W": "U3"},
    "RU4_BAY": {"N": None, "E": None, "S": None, "W": "U4"},
    "RU5_BAY": {"N": None, "E": None, "S": None, "W": "U5"},
    "RU6_BAY": {"N": None, "E": None, "S": None, "W": "U6"},
    "LU1_BAY": {"N": None, "E": "U1", "S": None, "W": None},
    "LU2_BAY": {"N": None, "E": "U2", "S": None, "W": None},
    "LU3_BAY": {"N": None, "E": "U3", "S": None, "W": None},
    "LU4_BAY": {"N": None, "E": "U4", "S": None, "W": None},
    "LU5_BAY": {"N": None, "E": "U5", "S": None, "W": None},
    "LU6_BAY": {"N": None, "E": "U6", "S": None, "W": None},
    "RB1_BAY": {"N": None, "E": "RB1", "S": None, "W": None},
    "RB2_BAY": {"N": None, "E": "RB2", "S": None, "W": None},
    "RB3_BAY": {"N": None, "E": "RB3", "S": None, "W": None},
    "RB4_BAY": {"N": None, "E": "RB4", "S": None, "W": None},
    "RB5_BAY": {"N": None, "E": "RB5", "S": None, "W": None},
    "RB6_BAY": {"N": None, "E": "RB6", "S": None, "W": None},
    "LB1_BAY": {"N": None, "E": None, "S": None, "W": "LB1"},
    "LB2_BAY": {"N": None, "E": None, "S": None, "W": "LB2"},
    "LB3_BAY": {"N": None, "E": None, "S": None, "W": "LB3"},
    "LB4_BAY": {"N": None, "E": None, "S": None, "W": "LB4"},
    "LB5_BAY": {"N": None, "E": None, "S": None, "W": "LB5"},
    "LB6_BAY": {"N": None, "E": None, "S": None, "W": "LB6"},
}


def exits(junction):
    """The compass directions that actually lead somewhere from a junction."""
    return tuple(d for d, target in JUNCTIONS[junction].items()
                 if target is not None)


def is_dead_end(junction):
    """True for a bay: a spur with nothing beyond it.

    The AGV knows it has arrived at one when the line simply runs out, so
    this replaces naming particular bay nodes in the route logic.
    """
    return len(exits(junction)) <= 1


def path_find(start_junction, end_junction):
    """Return shortest route steps as (junction, direction, next_junction).

    For example, path_find("HOME", "LB1") returns:
    [("HOME", "W", "LH"), ("LH", "W", "L1"),
     ("L1", "N", "L2"), ("L2", "N", "LB1")].
    """
    if start_junction not in JUNCTIONS:
        raise ValueError("Unknown start junction: {}".format(start_junction))
    if end_junction not in JUNCTIONS:
        raise ValueError("Unknown end junction: {}".format(end_junction))
    if start_junction == end_junction:
        return []

    # Breadth-first search finds a route with the fewest junction-to-junction steps.
    queue = [start_junction]
    # Record how each junction was first reached, preventing revisits.
    parent = {start_junction: None}
    # Keep the direction and previous junction needed to reconstruct each step.
    parent_step = {}
    queue_index = 0

    while queue_index < len(queue):
        junction = queue[queue_index]
        queue_index += 1

        # Explore every connected neighbor in NESW map order.
        for direction, neighbor in JUNCTIONS[junction].items():
            if neighbor is None or neighbor in parent:
                continue

            parent[neighbor] = junction
            parent_step[neighbor] = (junction, direction, neighbor)
            if neighbor == end_junction:
                # Follow the recorded links backward, then reverse to start order.
                steps = []
                current = end_junction
                while current != start_junction:
                    steps.append(parent_step[current])
                    current = parent[current]
                steps.reverse()
                return steps

            queue.append(neighbor)

    raise ValueError(
        "No route from {} to {}".format(start_junction, end_junction))


def _read_direction():
    """Read one WASD key and return its direction or Q."""
    import msvcrt

    return {
        b"w": "N",
        b"d": "E",
        b"s": "S",
        b"a": "W",
        b"q": "Q",
    }.get(msvcrt.getch().lower())


def navigate_map():
    """Navigate the junction map interactively from HOME."""
    current = "HOME"
    print("Map navigation: use WASD to move; Q to quit.")
    print("Current junction: {}".format(current))

    while True:
        exits = [
            "{} -> {}".format(direction, neighbor)
            for direction, neighbor in JUNCTIONS[current].items()
            if neighbor is not None
        ]
        print("Available exits: {}".format(", ".join(exits) or "none"))

        direction = _read_direction()
        if direction == "Q":
            print("Leaving map navigation.")
            return

        if direction is None:
            print("Unknown key; use WASD or Q to quit.")
            continue

        neighbor = JUNCTIONS[current][direction]
        if neighbor is None:
            print("No junction {} of {}.".format(direction, current))
            continue

        current = neighbor
        print("Current junction: {}".format(current))

#--------------------------------------------------------------------------------------------
def find_target_direction(current_junction, target_junction):
    "assume knowledge that target is adjacent to current"
    direction = next(k for (k, v) in JUNCTIONS[current_junction].items() if v == target_junction)
    return direction

#--------------------------------------------------------------------------------------------

if __name__ == "__main__":
    #print(path_find("RB6", "U6"))
    navigate_map()
    #print(find_target_direction("HOME","RH"))
