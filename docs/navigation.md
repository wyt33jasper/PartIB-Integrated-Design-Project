# How the AGV decides where to go

M1 drove a route typed out in `config.py` — a list of junctions, one turn at
a time. It worked once, and then had to be retyped for every change of plan.
Nothing in the system now names a destination.

## What is measured, and what is worked out

Four things decide where a package ends up. Only three are measured, and the
fourth is computed:

| | | from |
|---|---|---|
| **which rack** | A or B | which loading bay it was collected at — the AGV drove there, so this costs nothing |
| **which deck** | top or bottom | the magnet pole, read at the gripper |
| **which bay** | 1–6 | the side rangefinders, read from the corridor |
| **the route** | | `map.path_find()` |

The spec fixes the first two:

> Boxes collected from the left-hand areas go to 'A' rack, those on the
> right-hand area to 'B' rack. The boxes with north pole magnets need to be
> delivered to the top racks, those with south pole magnets to the bottom
> racks.

`nav/destination.py` is that rule and nothing else.

## The delivery leg

The AGV cannot know which bays are already full — the racks may be loaded
before the run starts, and the spec says to *only deliver if the location is
not already occupied*. So it does not pick a bay number in advance. It drives
the corridor serving the destination shelf and asks the rangefinder on the
relevant side at each junction:

```
corridor entry --+------------+------------+------------+--> shelf full,
                 |            |            |            |    carry it home
              occupied     occupied       FREE          |
              drive on     drive on      turn in        |
                                         deliver
```

A skipped bay costs only the width of a junction, so being wrong about a full
one is cheap. Being wrong about a free one stacks a package on another, which
is why `BAY_UNKNOWN_IS_OCCUPIED` defaults to skipping anything unreadable.

**The side to scan needs no table.** A bay sits on whichever side the AGV
would turn to enter it, so `State.turn_to(bay)` answers both questions at
once. Everything else about the shelves comes out of the map too — which
corridor serves them, and which junction each bay hangs off — so moving a bay
on the map moves it here.

Working it out rather than tabulating it gives:

| collected | pole | shelf | corridor | bays on the |
|---|---|---|---|---|
| left | north | A top | U1→U6 | left |
| right | north | B top | U1→U6 | right |
| left | south | A bottom | LB1→LB6 | right |
| right | south | B bottom | RB1→RB6 | left |

Note the top decks share one corridor with bays on both sides, and the bottom
decks are two separate corridors with bays on one side each. Nothing in the
code had to be told that.

## Why the sensors are split the way they are

**Distance sensors on the sides, Hall sensors at the gripper.**

Side-facing Hall sensors cannot do bay occupancy. A unipolar Hall only
answers for the pole it faces, and the upper racks hold north packages while
the lower ones hold south — so a north-tuned sensor would read every lower
bay as empty and the AGV would stack on top of a package.

The Hall pair at the gripper does two jobs at once. Neither sensor reads high
until the magnet is in range, so *"a pole is readable"* and *"the package is
close enough to grip"* are the same signal. Nothing else has to measure the
approach.

Worth testing early: the trip distance depends on magnet strength, and the
spec warns the blocks vary. Measure it across several packages before relying
on it as the grab trigger.

## Two things that look like details and are not

### Not every map node produces a junction event

Markings are painted where a branch leaves the line, so a node is only
visible to the outer trackers if it has an exit to one side of the direction
being travelled. `UU`, `LH` and `RH` are just line — the AGV drives straight
over them without an event. The planner walks its state through them rather
than expecting one event per map node.

`map.has_marking(node, heading)` decides this, which means the sensors and
the navigation cannot drift apart when the map is edited.

A bay produces no event either: the line simply runs out there.

### The sensor triggers the turn, not the junction

The side rangefinders point out of the chassis and watch the whole time the
AGV is moving, so the question they answer is simply *"is the bay beside me
free?"* — asked on every main-loop pass. When the answer is yes, the turning
function runs. The AGV never stops at a bay to measure it; it drives past the
full ones and turns into the first free one.

A junction is only a position update. It says **which** bay the AGV is beside,
not whether to take it:

```
junction fires      →  this is bay N, clear the window
every pass          →  does bay N read free?  yes → turn in
corridor runs out   →  shelf full, carry it home
```

The window is cleared on arriving beside each bay because the readings taken
on the approach are of an empty corridor, which reads the same as an empty
bay. **This is not a pause** — the AGV keeps driving, and the scan simply
stays silent for the `RANGER_CONFIRM_READS × RANGER_POLL_MS` it takes to
gather samples of that bay, about two centimetres of travel.

Counting *near readings* rather than averaging distances is deliberate: an
occupied bay returns the box face from part of the window and the rack frame
from the rest, so the mean sits between the two and means nothing.

The one timing constraint left:

```
RANGER_WINDOW_SAMPLES × RANGER_POLL_MS  ≪  time to pass one bay
```

Too long and a reading blends the bay with the frame beside it; too short and
one stray sample swings it.

## Replanning, not following

A route is planned at **every junction**, not once per leg. A breadth-first
search over sixty nodes is far cheaper than a main-loop period, and it buys
the property that matters on the table: if the AGV ends up somewhere it did
not intend to be, the next junction routes it onward from where it *actually
is*, instead of following a plan that no longer describes reality.

## The clock

The run is five minutes, and +15 goes to being back in the start box once at
least two packages are delivered — the same as one more correct delivery. The
decision *"should I start another package?"* is only made at one moment — a
bay has been dealt with — so the reserve has to cover the whole of what it is
deciding to start: collect, deliver, and get home afterwards.

Reserving only the drive home lets the AGV commit to a round trip it cannot
finish. In simulation at 6 s per junction that overran the limit by 69 s.

The reserve is also **measured, not trusted**. `CYCLE_RESERVE_MS` is a guess
made before the run; once a package has actually been timed, the slowest one
observed is the better estimate, so whichever is larger wins. A guess that is
too small costs one round trip, and then the measurement corrects it.

## Wiring

The Hall pair moved off GP10/GP11, which I2C1 now needs for the right-hand
rangefinder.

```
GP0,1     gripper actuator      GP18,19   line trackers
GP4,5     left motor            GP20      button
GP6,7     right motor           GP21,22   red / green pole lamps
GP10,11   I2C1 - right ranger   GP26      amber beacon
GP12,14   Hall north / south    GP27,28   turn trackers
GP16,17   I2C0 - left ranger
free: GP9, GP13, GP15 (servo), GP2,3
```

Two VL53L0Xs both answer at `0x29`, so they cannot share one bus without
XSHUT sequencing at boot. They go on separate buses instead — no boot-time
address juggling. A VL53L0X and a TMF8x01 (`0x41`) would share one bus
happily, if the team ends up with one of each, which would free GP10/GP11.

There is no QR reader. The brief mentions one in passing, but the scoring
only pays for identification "by north/south pole (red/green LED)", and the
rack-and-deck rule already fixes the destination completely from the pole
and the collection side. See the note in `sw/nav/destination.py`.

Use **time-of-flight, not ultrasonic**, for the sides. A sideways ultrasonic
cone sees the rack frame rather than the package, and 1 cm resolution is
marginal at these distances.

## Still to measure

Everything marked `CHECK` in `config.py`, but these three decide whether the
design works at all:

1. **Occupied vs empty reading** at the real bays. Two numbers: what the
   sensor reads with a package in the bay, and with the bay empty. The gap
   between them is how much margin the scan has. If an empty bay returns
   nothing at all, this is trivially reliable.
2. **`RANGER_WINDOW_SAMPLES`** — time how long one bay takes to pass at
   `FOLLOW_SPEED` and keep the window well inside it. Also check with
   `sw/test_rangefinder.py` whether the verdict flickers to empty in the gap
   *between* bays: if it does, the AGV could turn where there is no spur, and
   the fix is to require the bay's junction marking to have been seen first.
3. **`CYCLE_RESERVE_MS`** — time one complete collect-and-deliver cycle.

## Checking it without a robot

```bash
python3 tools/test_map.py       # the map is consistent and every route exists
python3 tools/test_mission.py   # the planner, against the real map
python3 tools/simulate_run.py   # boots sw/main.py and drives a whole run
```

`test_mission.py` keeps its own idea of where the AGV physically is,
independent of the mission's, and checks the two agree after every event.
That is the failure this design is vulnerable to: a route one junction out of
step is wrong everywhere after that point, and on the table it looks like a
mechanical problem.

`simulate_run.py` also reports the pin assignment as `main()` actually built
it, which is how a pin claimed twice gets caught. Set `NODE_TICKS` to change
how fast the simulated AGV travels.
