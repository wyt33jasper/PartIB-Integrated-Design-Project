"""Simulate a crossroad while the spacebar is held on a desktop.

Run with `python test_junction_sim.py` from the sw directory. This tests the
junction detector and route instruction only; it does not access robot hardware.
Keep the console focused and hold Space for at least the confirmation interval.
Release it for at least the clear interval before simulating another junction.
"""

import ctypes
import sys
from time import sleep

from config import (INITIAL_DIRECTION, INITIAL_NODE, JUNCTION_CLEAR_MS,
                    JUNCTION_CONFIRM_MS, SEQUENCE)
from nav.junction import NONE, JunctionDetector, name
from nav.navigator import State


class FakeSensors:
    """Report both turn trackers active only while the spacebar is held."""

    def __init__(self, key_is_pressed):
        self._key_is_pressed = key_is_pressed
        self.left = False
        self.right = False

    @staticmethod
    def spacebar_is_pressed():
        if sys.platform != "win32":
            raise RuntimeError("The keyboard simulator currently requires Windows")
        return bool(ctypes.windll.user32.GetAsyncKeyState(0x20) & 0x8000)

    def turn_trackers(self):
        pressed = self._key_is_pressed()
        self.left = pressed
        self.right = pressed
        return self.left, self.right


def main():
    sensors = FakeSensors(FakeSensors.spacebar_is_pressed)
    detector = JunctionDetector(sensors,
                                confirm_ms=JUNCTION_CONFIRM_MS,
                                clear_ms=JUNCTION_CLEAR_MS)
    state = State(INITIAL_NODE, INITIAL_DIRECTION, SEQUENCE)

    print("Hold Space to simulate a crossroad; Ctrl+C to stop.")
    print("Release Space between junctions so the detector can re-arm.")
    print("Sequence is", state.sequence)

    while True:
        signature = detector.poll()
        if signature != NONE:

            prev_node = state.current_node

            turn = state.junction_turn()

            if turn is not None:
                #turn action
                print("Junction detected: {} - route instruction: {}".format(
                    name(signature), turn))
                print("At {} going {}, action completed now heading {} to {}".format(prev_node, turn, state.current_direction, state.current_node))
            else:
                print("Junction detected: {} - route complete".format(       
                    name(signature)))
                break
        else:
            #line follow
            pass

        sleep(0.05)


if __name__ == "__main__":
    main()
