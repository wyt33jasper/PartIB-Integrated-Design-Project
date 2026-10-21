"""Host-side tests for the magnet sensor, pole indicator and destination rule.

    python3 tools/test_magnet_logic.py

Runs the real sw/ modules against the picostub clock and a fake ADC, so the
whole identify-and-route chain is checked without a robot or a magnet.

The destination checks also cross-reference nav/map.py: every bay node the
rule can produce has to exist on the map, and the loading bays have to be
real nodes too. That catches a naming mismatch here rather than on the table.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "sw"))

import utime                                            # noqa: E402
from machine import ADC, Pin                            # noqa: E402
import config                                           # noqa: E402
from drivers.magnet import (NORTH, SOUTH, AnalogueHall,  # noqa: E402
                            DigitalHallPair, MagnetSensor, probe_from_config)
from drivers.indicators import PoleIndicator            # noqa: E402
from nav import destination as dest                     # noqa: E402
from nav.map import JUNCTIONS                           # noqa: E402

failures = []


def check(what, got, want):
    ok = got == want
    if not ok:
        failures.append("{} (got {!r}, wanted {!r})".format(what, got, want))
    print("  {}  {:<48} {!r}".format("ok  " if ok else "FAIL", what, got))


ADC_PIN = 27            # a pin that would be free if a turn tracker moved


def make_analogue(**kw):
    """An AnalogueHall on a notional free ADC pin, for the analogue tests."""
    opts = dict(adc_pin=ADC_PIN,
                deadband=config.MAGNET_DEADBAND,
                north_is_high=config.MAGNET_NORTH_IS_HIGH)
    opts.update(kw)
    probe = AnalogueHall(**opts)
    sensor = MagnetSensor(probe,
                          poll_ms=config.MAGNET_POLL_MS,
                          confirm_reads=config.MAGNET_CONFIRM_READS)
    return sensor, probe, ADC.registry[ADC_PIN]


def make_digital():
    """The probe the M1 wiring actually supports."""
    probe = DigitalHallPair(config.MAGNET_NORTH_PIN, config.MAGNET_SOUTH_PIN,
                            active_high=config.MAGNET_ACTIVE_HIGH)
    sensor = MagnetSensor(probe,
                          poll_ms=config.MAGNET_POLL_MS,
                          confirm_reads=config.MAGNET_CONFIRM_READS)
    active = 1 if config.MAGNET_ACTIVE_HIGH else 0

    def present(pole):
        Pin.registry[config.MAGNET_NORTH_PIN].value(
            active if pole == NORTH else 1 - active)
        Pin.registry[config.MAGNET_SOUTH_PIN].value(
            active if pole == SOUTH else 1 - active)
    return sensor, probe, present


def main():
    print("digital pair - what the M1 wiring supports")
    sensor, probe, present = make_digital()
    present(NORTH)
    check("north sensor high reads north", probe.read(), NORTH)
    present(SOUTH)
    check("south sensor high reads south", probe.read(), SOUTH)
    present(None)
    check("neither high is no package", probe.read(), None)
    active = 1 if config.MAGNET_ACTIVE_HIGH else 0
    Pin.registry[config.MAGNET_NORTH_PIN].value(active)
    Pin.registry[config.MAGNET_SOUTH_PIN].value(active)
    check("both high is rejected, not guessed", probe.read(), None)

    print("\nthe probe chosen by config.py")
    chosen = probe_from_config(config)
    check("config selects the digital pair",
          isinstance(chosen, DigitalHallPair), True)
    raised = False
    try:
        AnalogueHall(None, config.MAGNET_DEADBAND,
                     config.MAGNET_NORTH_IS_HIGH)
    except ValueError:
        raised = True
    check("analogue refuses to build with no ADC pin assigned", raised, True)

    print("\nanalogue classification against the deadband")
    sensor, probe, adc = make_analogue()
    centre = probe.centre
    band = config.MAGNET_DEADBAND
    check("idle reading is no pole", probe.classify(centre), None)
    check("just inside the deadband is no pole",
          probe.classify(centre + band - 1), None)
    check("just outside, high, is north", probe.classify(centre + band), NORTH)
    check("just outside, low, is south", probe.classify(centre - band), SOUTH)
    check("far high is north", probe.classify(centre + 20000), NORTH)
    check("far low is south", probe.classify(centre - 20000), SOUTH)

    print("\nsensor mounted the other way round")
    _, flipped, _ = make_analogue(north_is_high=False)
    check("high now reads south", flipped.classify(flipped.centre + band), SOUTH)
    check("low now reads north", flipped.classify(flipped.centre - band), NORTH)

    print("\ncalibration replaces the assumed idle point")
    sensor, probe, adc = make_analogue()
    adc.value_u16 = 30000
    check("calibrate measures the real idle", sensor.calibrate(8), 30000)
    check("a reading at the new centre is no pole",
          probe.classify(30000), None)
    check("the old half-scale point is now off centre",
          probe.classify(32768 + band), NORTH)

    print("\nconfirmation - a pole is reported once, after 3 readings")
    sensor, probe, adc = make_analogue()
    adc.value_u16 = probe.centre + band + 500

    def poll_once():
        utime.advance(config.MAGNET_POLL_MS)
        return sensor.poll()

    check("read 1 of 3", poll_once(), None)
    check("read 2 of 3", poll_once(), None)
    check("read 3 of 3 reports north", poll_once(), NORTH)
    check("stays quiet afterwards", poll_once(), None)
    check("pole stays readable for the lamp", sensor.pole, NORTH)

    print("\nrate limit")
    before = sensor.confirmations
    sensor.poll()               # same millisecond
    check("a second call inside the window is ignored",
          sensor.confirmations, before)

    print("\nremoving the package clears the state")
    adc.value_u16 = probe.centre
    poll_once()
    check("no pole once the magnet leaves", sensor.pole, None)
    check("count cleared", sensor.confirmations, 0)

    print("\na brief spike cannot flip the reported pole")
    sensor, probe, adc = make_analogue()
    adc.value_u16 = probe.centre + band + 500
    poll_once(); poll_once(); poll_once()        # north confirmed
    adc.value_u16 = probe.centre - band - 500   # one transient the other way
    check("single opposite reading does not report", poll_once(), None)
    check("reported pole is still north", sensor.pole, NORTH)
    adc.value_u16 = probe.centre + band + 500
    poll_once()
    check("north survives the glitch", sensor.pole, NORTH)

    print("\na sustained change does flip it")
    adc.value_u16 = probe.centre - band - 500
    poll_once(); poll_once()
    check("two south readings are not enough", sensor.pole, NORTH)
    check("the third reports south", poll_once(), SOUTH)
    check("and it is now the reported pole", sensor.pole, SOUTH)

    print("\npole indicator")
    indicator = PoleIndicator(config.RED_LED_PIN, config.GREEN_LED_PIN)
    red = Pin.registry[config.RED_LED_PIN]
    green = Pin.registry[config.GREEN_LED_PIN]
    indicator.show(NORTH)
    check("north lights red only", (red.value(), green.value()), (1, 0))
    indicator.show(SOUTH)
    check("south lights green only", (red.value(), green.value()), (0, 1))
    indicator.show(None)
    check("nothing carried, both off", (red.value(), green.value()), (0, 0))
    indicator.show(NORTH)
    indicator.off()
    check("off() clears both", (red.value(), green.value()), (0, 0))

    print("\ndestination rule from the spec")
    check("left collection goes to rack A", dest.rack_for(dest.LEFT), dest.RACK_A)
    check("right collection goes to rack B", dest.rack_for(dest.RIGHT), dest.RACK_B)
    check("north goes to the top racks", dest.level_for(NORTH), dest.TOP)
    check("south goes to the bottom racks", dest.level_for(SOUTH), dest.BOTTOM)
    check("left + north", dest.destination(NORTH, dest.LEFT),
          (dest.RACK_A, dest.TOP))
    check("left + south", dest.destination(SOUTH, dest.LEFT),
          (dest.RACK_A, dest.BOTTOM))
    check("right + north", dest.destination(NORTH, dest.RIGHT),
          (dest.RACK_B, dest.TOP))
    check("right + south", dest.destination(SOUTH, dest.RIGHT),
          (dest.RACK_B, dest.BOTTOM))

    print("\nbad input is rejected rather than guessed")
    for call, what in [
        (lambda: dest.rack_for("middle"), "unknown pickup side"),
        (lambda: dest.level_for("sideways"), "unknown pole"),
        (lambda: dest.level_for(None), "no pole measured"),
        (lambda: dest.bay_nodes("C", dest.TOP), "rack that does not exist"),
        (lambda: dest.side_of_loading_bay("RB3_BAY"), "not a loading bay"),
    ]:
        raised = False
        try:
            call()
        except ValueError:
            raised = True
        check("raises on {}".format(what), raised, True)

    print("\nevery bay the rule can produce exists on the map")
    missing = []
    total = 0
    for rack in (dest.RACK_A, dest.RACK_B):
        for level in (dest.TOP, dest.BOTTOM):
            for node in dest.bay_nodes(rack, level):
                total += 1
                if node not in JUNCTIONS:
                    missing.append(node)
    check("storage bays checked", total, 24)
    check("none missing from map.py", missing, [])

    print("\nloading bays")
    check("four of them", len(dest.LOADING_BAYS), 4)
    check("all present on the map",
          [n for n in dest.LOADING_BAYS if n not in JUNCTIONS], [])
    check("L1_BAY collects to rack A",
          dest.rack_for(dest.side_of_loading_bay("L1_BAY")), dest.RACK_A)
    check("R2_BAY collects to rack B",
          dest.rack_for(dest.side_of_loading_bay("R2_BAY")), dest.RACK_B)

    print("\nstorage and loading bays do not overlap")
    storage = set()
    for rack in (dest.RACK_A, dest.RACK_B):
        for level in (dest.TOP, dest.BOTTOM):
            storage.update(dest.bay_nodes(rack, level))
    check("no node is both", sorted(storage & set(dest.LOADING_BAYS)), [])
    map_bays = {n for n in JUNCTIONS if n.endswith("_BAY")}
    check("together they account for every _BAY node on the map",
          sorted(map_bays - storage - set(dest.LOADING_BAYS)), [])

    print()
    if failures:
        print("{} FAILED".format(len(failures)))
        for f in failures:
            print("  -", f)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
