JUNCTIONS = {
    "HOME": {"N": "UU", 
             "E": "RH",
             "S": None,
             "W": "LH"},
    "UU":   {"N": "U1",
             "E": None,
             "S": "HOME",
             "W": None},
    "U1":   {"N": "U2",
             "E": None,
             "S": "UU",
             "W": None},
    "U2":   {"N": "U3",
             "E": None,
             "S": "U1",
             "W": None},
    "U3":   {"N": "U4",
             "E": None,
             "S": "U2",
             "W": None},
    "U4":   {"N": "U5",
             "E": None,
             "S": "U3",
             "W": None},
    "U5":   {"N": "U6",
             "E": None,
             "S": "U4",
             "W": None},
    "U6":   {"N": None,
             "E": None,
             "S": "U5",
             "W": None},

    "RH":   {"N": None,
             "E": "R1",
             "S": None,
             "W": "HOME"},
    "R1":   {"N": "R2",
             "E": None,
             "S": "RH",
             "W": None},
    "R2":   {"N": "RB1",
             "E": None,
             "S": "R1",
             "W": None},
    "RB1":  {"N": "RB2",
             "E": None,
             "S": "R2",
             "W": None},
    "RB2":  {"N": "RB3",
             "E": None,
             "S": "RB1",
             "W": None},
    "RB3":  {"N": "RB4",
             "E": None,
             "S": "RB2",
             "W": None},
    "RB4":  {"N":"RB5",
             "E": None,
             "S": "RB3",
             "W": None},
    "RB5":  {"N":"RB6",
             "E": None,
             "S": "RB4",
             "W": None},
    "RB6":  {"N": None,
             "E": None,
             "S": "RB5",
             "W": None},

    "LH":   {"N": None,
             "E": "HOME",
             "S": None,
             "W": "L1"},
    "L1":   {"N": "L2",
             "E": None,
             "S": "LH",
             "W": None},
    "L2":   {"N": "LB1",
             "E": None,
             "S": "L1",
             "W": None},
    "LB1":  {"N": "LB2",
             "E": None,
             "S": "L2",
             "W": None},
    "LB2":  {"N": "LB3",
             "E": None,
             "S": "LB1",
             "W": None},
    "LB3":  {"N": "LB4",
             "E": None,
             "S": "LB2",
             "W": None},
    "LB4":  {"N":"LB5",
             "E": None,
             "S": "LB3",
             "W": None},
    "LB5":  {"N":"LB6",
             "E": None,
             "S": "LB4",
             "W": None},
    "LB6":  {"N": None,
             "E": None,
             "S": "LB5",
             "W": None},

}


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


if __name__ == "__main__":
    print(path_find("RB6", "U6"))
    #navigate_map()
