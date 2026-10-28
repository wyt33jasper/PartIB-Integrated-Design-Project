# V3.5 - the tree that goes on the Pico

Blake's V3.5 (`blakehrobinson-design/IDP-repo@fd6d73d`) with the rear-sensor
bay search added. Everything else is byte-identical to his copy, including
every driver and the whole of `nav/` except `bay.py` and `mission.py`.

Changed: `main.py`, `config.py`, `nav/bay.py`, `nav/mission.py`,
`drivers/distance.py`, `test_bay.py`, `test_distance.py`.

## How a bay is searched

One VL53L0X at the back, on GP8/GP9. A side-facing sensor's cone is far too
wide to pick one bay out from across the corridor, so the AGV goes to the bay
instead:

1. at the shelf junction, rotate 90 deg so its back is to the bay
2. reverse up to the bay mouth, reading the rear sensor on the way
3. **full** - a near reading stops the reverse early; come out and carry on
   down the shelf
4. **empty** - a run of far readings while standing at the mouth; come out,
   spin 180 deg, find the bay's line and drive in to deliver

Steps 1 and 4 are timed rotations, not line-seeking turns: there is no line
running across the corridor for the trackers to stop on. That is the one open
-loop part of this, so tune it first.

## Tuning order

| # | Value | How |
|---|-------|-----|
| 1 | `BAY_FULL_MM` | `test_distance.py`. Back up to a bay, note the reading with a package and without. Put the threshold between them. |
| 2 | `BAY_FACE_AWAY_MS` | `test_bay.py`. Raise or lower until the first rotation is a square 90 deg. |
| 3 | `BAY_APPROACH_MS` | `test_bay.py`. Until the reverse stops at the bay mouth - close enough to read, not touching the rack. |
| 4 | `BAY_SPIN_MS` | `test_bay.py` on an **empty** bay. Until the AGV ends up square on the bay's line. `no line after the bay spin` means it is still wrong. |

`test_bay.py` runs the real manoeuvre once per button press and nothing else,
so it is safe to run with the rack loaded. Set `SIDE` at the top of it to
whichever side the bay is on.

## Pickup bays

They are not measured - nothing can see into one from the corridor. The AGV
drives into each in turn (`PICKUP_ORDER`), closes the gripper, and takes the
Hall sensor finding no pole as "no package here", then moves on to the next.

## Still unmeasured

Every value in the table above, plus `CYCLE_RESERVE_MS`. Run the two bench
tests before a full run - with the sensor unplugged or the threshold wrong,
every bay reads full and the AGV will drive the whole shelf without
delivering.
