"""Sweep FOLLOW_SPEED and FOLLOW_STEER through the simulator.

    python3 tools/tune.py

Prints a table of how well each pair of settings tracks the line, so the team
starts bench testing from sensible numbers instead of from a guess.

Read the columns like this:

    mean    average distance from the line centre - how tightly it tracks
    worst   the furthest it strays, usually mid-bend.  Keep this well under
            the point where both inner sensors leave the tape, or the AGV
            drops into the reverse branch during normal running
    lost    percentage of loop passes with no sensor on the line.  Anything
            above zero means the settings are not usable
    speed   distance covered in the run - the competition is scored on how
            much gets delivered in five minutes, so faster matters

These are simulator numbers from an idealised model with no wheel slip and no
sensor noise.  Use them to pick which settings to try first, then confirm on
the real table.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import simulate                                        # noqa: E402

SPEEDS = (40, 60, 80, 100)
STEERS = (15, 20, 25, 30, 40)
SECONDS = 40


def main():
    print(f"{SECONDS} s per run on a {simulate.build_track().__len__() / 1000:.2f} m "
          f"closed circuit, {simulate.LINE_HALF_WIDTH_MM * 2:.0f} mm tape\n")
    print(f"{'speed':>5} {'steer':>6} | {'mean':>7} {'worst':>7} {'lost':>6} "
          f"{'travelled':>10}  verdict")
    print("-" * 68)

    best = None
    for speed in SPEEDS:
        for steer in STEERS:
            r = simulate.run(speed=speed, steer=steer, seconds=SECONDS)

            if r["lost_pct"] > 0:
                verdict = "loses the line"
            elif r["worst_offset_mm"] > simulate.LINE_HALF_WIDTH_MM * 1.6:
                verdict = "strays wide on bends"
            else:
                verdict = "ok"
                # Prefer the fastest setting that still tracks tightly.
                key = (-r["travelled_m"], r["mean_offset_mm"])
                if best is None or key < best[0]:
                    best = (key, speed, steer, r)

            print(f"{speed:>5} {steer:>6} | {r['mean_offset_mm']:>6.1f}m "
                  f"{r['worst_offset_mm']:>6.1f}m {r['lost_pct']:>5.1f}% "
                  f"{r['travelled_m']:>9.2f}m  {verdict}")
        print()

    if best:
        _, speed, steer, r = best
        print(f"best tracking pair : FOLLOW_SPEED = {speed}, FOLLOW_STEER = {steer}")
        print(f"                     mean {r['mean_offset_mm']:.1f} mm, "
              f"worst {r['worst_offset_mm']:.1f} mm, "
              f"{r['travelled_m']:.2f} m in {SECONDS} s")
    else:
        print("no setting tracked cleanly - check the geometry constants")


if __name__ == "__main__":
    main()
