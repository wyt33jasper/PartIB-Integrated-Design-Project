# Table survey

Fill this in at the table, with the AGV in hand.  Everything in `nav/route.py`
is generated from what is written here, so a mistake on this page becomes a
wrong delivery in the competition.

Topological navigation needs no distances.  Do not measure anything except
where a note explicitly asks for it.

---

## 1. Junctions

Walk the whole line network.  Every place the line splits, crosses or ends is
a junction.  Give each one a short id and keep using it.

Pick ids that mean something — `START`, `J1`, `J2`, `RAMP_FOOT`, `A_UPPER` —
not numbers alone.

| id | where it is | notes |
|----|-------------|-------|
|  |  |  |
|  |  |  |
|  |  |  |
|  |  |  |
|  |  |  |
|  |  |  |
|  |  |  |
|  |  |  |

## 2. What each junction looks like

This is what the two **outer** turn trackers see as the AGV arrives.  It is
the AGV's only way to confirm it is where it thinks it is.

A junction can look different depending on which way you arrive, so record one
row per approach direction.

Signature is one of: `left`, `right`, `crossroads`.

| junction | arriving from | signature | exits available |
|----------|---------------|-----------|-----------------|
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |

Watch for junctions that look identical — those are where the AGV will lose
sync, and they are worth knowing about before the competition.

## 3. How the junctions connect

One row per line segment.  `from` and `to` are junction ids, `exit` is which
way the AGV leaves `from` to get to `to`.

Exit is one of: `left`, `right`, `straight`.

| from | exit | to | notes (ramp? narrow? bad tape?) |
|------|------|----|---------------------------------|
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |

## 4. Landmarks

Which junction, or which segment, is each of these?

| landmark | junction / segment | notes |
|----------|--------------------|-------|
| start / finish box |  |  |
| traffic light (stop line) |  | which approach? |
| pickup bay 1 |  |  |
| pickup bay 2 |  |  |
| pickup bay 3 |  |  |
| pickup bay 4 |  |  |
| rack A upper |  |  |
| rack A lower |  |  |
| rack B upper |  |  |
| rack B lower |  |  |
| foot of the ramp |  |  |
| top of the ramp |  |  |

Left and right in the task description are **as seen looking at the traffic
lights** — bays on the left go to rack A, bays on the right to rack B.  Note
which physical side that is, so the route table is not mirrored.

## 5. Things to measure (only these)

| what | value | why |
|------|-------|-----|
| tape width | mm | sets the inner sensor spacing |
| narrowest gap between two junctions | mm | sets `JUNCTION_CLEAR_MS` |
| tightest bend radius | mm | sets the top usable `FOLLOW_SPEED` |
| ramp angle | deg | does the gripper ground out? |
| is the line on the ramp continuous? | yes / no |  |

## 6. Anything that will break the assumptions

Free text.  Gaps in the tape, junctions the outer sensors cannot see, places
where two lines run close enough to confuse the inner pair, shiny patches,
anywhere the AGV could leave the table.

```
```
