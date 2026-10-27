"""Drive the real line follower around a virtual table.

This imports sw/nav/follower.py, sw/drivers/*.py and sw/config.py unchanged -
the only fakes are machine and utime, which the Pico provides for real.  So a
steering sign error, a bad speed constant or a broken branch shows up here
exactly as it would on the table, without needing a built robot.

    python3 tools/simulate.py

Writes tools/out/path.svg - open it in VS Code to see where the AGV went.

The model is deliberately simple: no wheel slip, no sensor noise, no motor
lag.  It proves the logic is right, not that the tuning is right.  Only the
real table can tell you that.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))   # fake machine / utime
sys.path.insert(0, os.path.join(REPO, "sw"))         # the real firmware

import utime                                          # noqa: E402
import config                                         # noqa: E402
from drivers.drive import Drive                       # noqa: E402
from drivers.line_sensor import LineSensorArray       # noqa: E402
from machine import Pin                               # noqa: E402
from nav.follower import LineFollower                 # noqa: E402

# --------------------------------------------------------------- AGV geometry
# Rough numbers for a chassis inside the 200 x 180 mm limit.  Adjust once the
# mechanical team has fixed the real layout.
WHEEL_BASE_MM = 120.0          # distance between the driven wheels
SENSOR_AHEAD_MM = 80.0         # sensor bar, forward of the wheel axis
LINE_TRACKER_OFFSET_MM = 8.0   # inner pair, each side of centre
TURN_TRACKER_OFFSET_MM = 35.0  # outer pair, each side of centre

# 40 RPM gearbox on a 60 mm wheel -> about 125 mm/s at full duty.
MAX_SPEED_MM_S = 125.0

# ------------------------------------------------------------------- the track
LINE_HALF_WIDTH_MM = 10.0      # 20 mm tape
STEP_MS = config.LOOP_PERIOD_MS
RUN_SECONDS = 60


def build_track():
    """A closed circuit: four straights joined by four 90 degree bends.

    Closed so the AGV keeps following instead of running out of line, which
    is what makes the offset statistics meaningful.  Returned as a dense
    polyline; the arena asks "is any point of this polyline within half a
    tape width of (x, y)?"
    """
    pts = []
    x, y, heading = 100.0, 100.0, 0.0

    def straight(length):
        nonlocal x, y
        for _ in range(int(length)):
            x += math.cos(heading)
            y += math.sin(heading)
            pts.append((x, y))

    def arc(radius, degrees):
        nonlocal x, y, heading
        steps = int(abs(math.radians(degrees)) * radius)
        d_heading = math.radians(degrees) / steps
        for _ in range(steps):
            heading += d_heading
            x += math.cos(heading)
            y += math.sin(heading)
            pts.append((x, y))

    for long_side in (True, False, True, False):
        straight(400 if long_side else 300)
        arc(150, 90)
    return pts


class Arena:
    def __init__(self, track_points):
        self._pts = track_points
        # Bucket the track by x so the lookup does not scan every point.
        self._buckets = {}
        for px, py in track_points:
            self._buckets.setdefault(int(px // 20), []).append((px, py))

    def is_white(self, x, y):
        key = int(x // 20)
        for k in (key - 1, key, key + 1):
            for px, py in self._buckets.get(k, ()):
                if (px - x) ** 2 + (py - y) ** 2 <= LINE_HALF_WIDTH_MM ** 2:
                    return True
        return False


# ------------------------------------------------------------------- the robot
class SimulatedAGV:
    def __init__(self, arena, x, y, heading, speed=None, steer=None):
        self.arena = arena
        self.x = x
        self.y = y
        self.heading = heading

        self.drive = Drive(config.LEFT_MOTOR_DIR_PIN, config.LEFT_MOTOR_PWM_PIN,
                           config.RIGHT_MOTOR_DIR_PIN,
                           config.RIGHT_MOTOR_PWM_PIN,
                           rotate_scale=config.TURN_ROTATE_SCALE)
        self.sensors = LineSensorArray(
            config.LINE_TRACKER_L_PIN,
            config.LINE_TRACKER_R_PIN,
            config.TURN_TRACKER_L_PIN if config.TURN_TRACKERS_FITTED else None,
            config.TURN_TRACKER_R_PIN if config.TURN_TRACKERS_FITTED else None,
            white_is_high=config.LINE_WHITE_IS_HIGH)
        self.follower = LineFollower(self.drive, self.sensors,
                                     speed=config.FOLLOW_SPEED if speed is None else speed,
                                     steer=config.FOLLOW_STEER if steer is None else steer,
                                     reverse_speed=config.RECOVER_SPEED,
                                     lost_timeout_ms=config.LOST_TIMEOUT_MS)

    def _sensor_xy(self, side_offset):
        """World position of a sensor mounted ahead of the axis and to one side."""
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        x = self.x + SENSOR_AHEAD_MM * cos_h - side_offset * sin_h
        y = self.y + SENSOR_AHEAD_MM * sin_h + side_offset * cos_h
        return x, y

    def _update_sensor_pins(self):
        placement = [
            (config.LINE_TRACKER_L_PIN, +LINE_TRACKER_OFFSET_MM),
            (config.LINE_TRACKER_R_PIN, -LINE_TRACKER_OFFSET_MM),
        ]
        if config.TURN_TRACKERS_FITTED:
            placement += [
                (config.TURN_TRACKER_L_PIN, +TURN_TRACKER_OFFSET_MM),
                (config.TURN_TRACKER_R_PIN, -TURN_TRACKER_OFFSET_MM),
            ]
        for pin_no, offset in placement:
            sx, sy = self._sensor_xy(offset)
            white = self.arena.is_white(sx, sy)
            # config.LINE_WHITE_IS_HIGH describes the module, so drive the raw
            # pin the way that module would.
            raw = 1 if white else 0
            if not config.LINE_WHITE_IS_HIGH:
                raw = 1 - raw
            Pin.registry[pin_no].value(raw)

    def _wheel_speed_mm_s(self, motor):
        duty = motor._pwm.duty / 65535.0
        forward = motor._dir.value() == 0
        if motor._reverse:
            forward = not forward
        return (duty if forward else -duty) * MAX_SPEED_MM_S

    def step(self, dt_s):
        self._update_sensor_pins()
        self.follower.tick()

        v_l = self._wheel_speed_mm_s(self.drive._left)
        v_r = self._wheel_speed_mm_s(self.drive._right)

        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / WHEEL_BASE_MM

        self.heading += omega * dt_s
        self.x += v * math.cos(self.heading) * dt_s
        self.y += v * math.sin(self.heading) * dt_s


# ---------------------------------------------------------------------- output
def write_svg(track, trail, out_file):
    xs = [p[0] for p in track] + [p[0] for p in trail]
    ys = [p[1] for p in track] + [p[1] for p in trail]
    pad = 60
    min_x, max_x = min(xs) - pad, max(xs) + pad
    min_y, max_y = min(ys) - pad, max(ys) + pad
    w, h = max_x - min_x, max_y - min_y

    def poly(points, step=1):
        return " ".join(f"{px - min_x:.1f},{py - min_y:.1f}"
                        for px, py in points[::step])

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.0f} {h:.0f}" width="{w:.0f}" height="{h:.0f}">
  <rect width="100%" height="100%" fill="#2f3335"/>
  <polyline points="{poly(track, 2)}" fill="none" stroke="#f2f2f2"
            stroke-width="{LINE_HALF_WIDTH_MM * 2}" stroke-linecap="round"/>
  <polyline points="{poly(trail, 4)}" fill="none" stroke="#19b8a0"
            stroke-width="3" stroke-dasharray="8 5"/>
  <circle cx="{trail[0][0] - min_x:.1f}" cy="{trail[0][1] - min_y:.1f}" r="7"
          fill="#19b8a0"/>
  <circle cx="{trail[-1][0] - min_x:.1f}" cy="{trail[-1][1] - min_y:.1f}" r="7"
          fill="#e5484d"/>
  <text x="14" y="26" fill="#f2f2f2" font-family="sans-serif" font-size="16">
    white = taped line, green dashes = AGV path, green dot = start, red dot = end
  </text>
</svg>
"""
    with open(out_file, "w") as f:
        f.write(svg)


def run(speed=None, steer=None, seconds=RUN_SECONDS, svg_path=None):
    """Run one simulation and return its statistics."""
    track = build_track()
    arena = Arena(track)

    # Start on the line, pointing along it.
    agv = SimulatedAGV(arena, track[0][0], track[0][1], 0.0,
                       speed=speed, steer=steer)

    trail = []
    dt_s = STEP_MS / 1000.0
    steps = int(seconds * 1000 / STEP_MS)

    max_error = 0.0
    error_sum = 0.0
    lost_steps = 0

    for _ in range(steps):
        agv.step(dt_s)
        utime.advance(STEP_MS)
        trail.append((agv.x, agv.y))

        # Cross-track error: distance from the AGV centre to the nearest point
        # of the taped line.
        err = min(math.hypot(px - agv.x, py - agv.y) for px, py in track)
        max_error = max(max_error, err)
        error_sum += err
        if not arena.is_white(*agv._sensor_xy(+LINE_TRACKER_OFFSET_MM)) and \
           not arena.is_white(*agv._sensor_xy(-LINE_TRACKER_OFFSET_MM)):
            lost_steps += 1

    travelled = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(trail, trail[1:]))

    if svg_path:
        os.makedirs(os.path.dirname(svg_path), exist_ok=True)
        write_svg(track, trail, svg_path)

    return {
        "track_m": len(track) / 1000.0,
        "travelled_m": travelled / 1000.0,
        "mean_offset_mm": error_sum / steps,
        "worst_offset_mm": max_error,
        "lost_pct": 100.0 * lost_steps / steps,
        "steps": steps,
    }


def main():
    out_file = os.path.join(HERE, "out", "path.svg")
    r = run(svg_path=out_file)

    print(f"simulated          : {RUN_SECONDS} s at {STEP_MS} ms per loop pass")
    print(f"speed / steer      : {config.FOLLOW_SPEED} / {config.FOLLOW_STEER}")
    print(f"track length       : {r['track_m']:.2f} m (closed circuit)")
    print(f"distance travelled : {r['travelled_m']:.2f} m")
    print(f"mean offset        : {r['mean_offset_mm']:.1f} mm from the line centre")
    print(f"worst offset       : {r['worst_offset_mm']:.1f} mm  "
          f"(tape half width is {LINE_HALF_WIDTH_MM:.0f} mm)")
    print(f"line lost          : {r['lost_pct']:.1f}% of loop passes")
    print(f"written            : {out_file}")


if __name__ == "__main__":
    main()
