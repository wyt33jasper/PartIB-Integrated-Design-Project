# v3.5 — the tree that goes on the robot

Blake's V3.5 from `blakehrobinson-design/IDP-repo`, **unchanged**, plus
empty-bay detection. This is what the physical robot is built around, so
this is what gets uploaded to the Pico.

His repo was never modified — only read.

## What was added

| | |
|---|---|
| `sw/nav/bay.py` | `BayDetector` — works like `nav/junction.py` |
| `sw/drivers/distance.py` | the two VL53L0X, read without blocking |
| `sw/test_bay.py` | bench test: drives past the racks and reports each bay |
| `sw/config.py` | a `DISTANCE_*` / `BAY_*` block at the end |

`sw/main.py` has five additions and nothing removed:

- three imports
- `run()` takes `bay` and `bay_side`
- the bay scan block, which does nothing while `bay_side` is `None`
- one `bay.cancel()` in the route-turn branch
- `main()` builds the sensors

**With `bay_side = None` the loop behaves exactly as Blake wrote it.**
Verified on the host: 50 passes with the scan disarmed never touch a
sensor, and `cancel()` on an idle detector changes nothing.

## Pins

```
GP4,5 / GP6,7   motors          GP18,19   line trackers
GP8,9           distance left   GP20      button
GP10,11         distance right  GP22,17   turn trackers
GP13, GP15      lifter, grabber GP26      amber beacon
```

Nothing is claimed twice. GP8-11 were free in V3.5.

## Arming the scan

```python
run(..., bay, bay_side="left")     # or "right", or None to leave it off
```

Set it when the AGV enters a rack corridor; it clears itself the moment a
bay is taken. Navigation decides when and which side — this only measures.

## Before the first run

`BAY_FULL_MM = 120` is a guess. Run `sw/test_bay.py`: put a box in one
bay, leave the next empty, and read the two distances it prints. The
threshold goes between them.
