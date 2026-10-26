# Empty bay detection

Skips full bays and stops at the first empty one, while driving.

## Two files

| | |
|---|---|
| `sw/drivers/distance.py` | the two VL53L0X, read without blocking |
| `sw/nav/bay.py` | `BayDetector` — works like `nav/junction.py` |

## How it decides when to look

The distance sensors are mounted level with the junction (line) sensors. So
a junction marking means **a bay is beside us right now** — that is the
moment to measure, and the only moment.

Measuring continuously down the corridor does not work: the rack's own
dividers are close to the sensor and read as full bays. Measuring only at
the junction skips them.

The robot is never stopped. `start()` begins the measurement and `poll()`
finishes it over the next few passes while the line follower keeps running.

## Using it in main()

```python
from nav.bay import BayDetector, EMPTY, NONE, LEFT, RIGHT

# in main(), after the junction detector
left, right = build_pair(config)
bay = BayDetector(left, right,
                  full_mm=config.BAY_FULL_MM,
                  confirm_reads=config.BAY_CONFIRM_READS,
                  timeout_ms=config.BAY_TIMEOUT_MS)

# in the loop
if junction_detector.poll() != NO_JUNCTION:
    bay.start(rack_side)           # LEFT or RIGHT

if bay.poll() == EMPTY:            # or: if bay.bay_is_empty():
    turn = "left" if rack_side == LEFT else "right"
    _perform_turn(button, beacon, drive, sensors, turn, state)
```

`bay_is_empty()` exists because it was asked for by name — it returns the
same answer as the last `poll()` event, so either reads fine.

Call `bay.cancel()` when a turn starts, so a half-finished measurement is
not carried into the next bay.

## Which side

`rack_side` is whichever side the destination rack is on — the side you
would turn toward. Navigation owns that choice; the detector just measures
the side it is given, and never touches the other sensor.

## The verdict rule

- **one** near reading → **full**, decided immediately
- `BAY_CONFIRM_READS` far readings → **empty**
- nothing readable before `BAY_TIMEOUT_MS` → **full**

Asymmetric on purpose. A box is the only thing that should be close, so one
sighting is enough. Being wrong about "full" costs one bay; being wrong
about "empty" drops a package on top of another.

## To measure on the table

Run `sw/test_bay.py`. It follows the line and prints each bay as empty or
full with the distance, without turning.

1. Put a box in one bay, leave the next empty.
2. Note the two distances. `BAY_FULL_MM` goes between them.
3. If every bay reads full, either a sensor is not answering or
   `BAY_TIMEOUT_MS` is shorter than the time one bay takes to pass.

## Pins

```
GP8  / GP9    I2C0   left sensor
GP10 / GP11   I2C1   right sensor
```

Separate buses because both VL53L0X answer at `0x29`.
