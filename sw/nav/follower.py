"""Line following, as drawn in the team flowchart.

Two levels, matching the two charts:

  line_follower()           reads the outer "turn tracker" pair.  0/0 means
                            there is no junction here, so hand over to the
                            straight line follower.  Anything else means a
                            junction marking is in view and the turn decider
                            chooses what to do with it.

  straight_line_follower()  reads the inner "line tracker" pair and steers:

                              L/R   action
                              1/1   both motors forward
                              0/1   turn right
                              1/0   turn left
                              0/0   reverse

Neither function blocks.  Each call applies one correction and returns, so
the stop button is tested between every pair of steering decisions.  The
turns are continuous corrections, not fixed-angle manoeuvres - holding the
correction for a whole turn would leave the AGV blind to the stop button.

Week 1 scope is line following only, so turn_decider() is deliberately a stub
that falls through to the default line follower.  Navigation plugs in there
later without touching anything else.
"""

from utime import ticks_ms, ticks_diff

# Decisions turn_decider() can return. number is just a proxy 
STRAIGHT = 0
REVERSE = 1
ROTATE_180 = 2
ROTATE_90_Left = 3
ROTATE_90_Right = 4

class LineFollower:
    def __init__(self, drive, sensors,
                 speed=60, steer=40, reverse_speed=35, lost_timeout_ms=2000, orientation="NORTH", junction_prev="HOME", junction_next="HOME"):
        self._drive = drive
        self._sensors = sensors
        self._speed = speed
        self._steer = steer
        self._reverse_speed = reverse_speed
        self._lost_timeout_ms = lost_timeout_ms
        self._lost_since_ms = None
        self.in_action = 0

    # ------------------------------------------------------------- top level
    def tick(self):
        """One pass of the line follower.  Must not block."""

        line_l, line_r = self._sensors.line_trackers()
        turn_l, turn_r = self._sensors.turn_trackers()

        if self.in_action:
            return line_l, line_r, turn_l, turn_r

        decision = self.turn_decider(line_l, line_r, turn_l, turn_r)
        if decision == STRAIGHT:
            self._straight_line_follower()
        elif decision == REVERSE:
            self._line_lost()
        elif decision == ROTATE_180:
            self._rotate_180()
        elif decision == ROTATE_90_Left:
            self._rotate_90_left()
        elif decision == ROTATE_90_Right:
            self._rotate_90_right()
        else:
            self._straight_line_follower()      # default line follower

    def turn_decider(self, line_l, line_r, turn_l, turn_r):
        """Choose what to do at a junction.

        STUB - week 1 only has to follow the line, so every junction is
        driven straight through.  Navigation will replace this with a lookup
        against the current route, using turn_l/turn_r to confirm that the
        expected junction has actually been reached.
        """

        if not line_l and not line_r:  #when the robot sees no line in front
            if not turn_l and not turn_r:
                # 0/0 line, 0,0 turn 
                return REVERSE
        
            elif turn_l and not turn_r:
                # 0/0 line, 1/0 turn 
                # rotate 90 left
                return ROTATE_90_Left

            elif not turn_l and turn_r:
                # 0/0 line, 0/1 turn 
                # rotate 90 right
                return ROTATE_90_Right

            if turn_l and turn_r:
                # 0/0 line, 1/1 turn 
                # rotate  90 left or 90 right or rotate 180
                ### default to turn 180
                return ROTATE_180

        else: # 1/1 or 1/0 or 0/1 line, when robot sees line in front
            return STRAIGHT  # default to go straight
        
        return STRAIGHT
    # -------------------------------------------------- straight line follower
    def _straight_line_follower(self):
        line_l, line_r = self._sensors.line_trackers()

        if line_l and line_r:                   # 1/1 - on track
            self._on_line()
            self._drive.move(self._speed, 0)
        elif not line_l and line_r:             # 0/1 - drifted, steer right
            self._on_line()
            self._turn_right()
        else:                                   # 1/0 - drifted, steer left
            self._on_line()
            self._turn_left()
        #else:                                   # 0/0 - line lost
        #    self._line_lost()                  ### taken care in turn decider

    # ------------------------------------------------------------ primitives
    def _turn_right(self):
        self._drive.move(self._speed, self._steer)

    def _turn_left(self):
        self._drive.move(self._speed, -self._steer)
    # ------------------------------------------------------------ actions
    # rotate(self, l_or_r, speed) is defined in drive.py
    """Rotate in place.  l_or_r: -1 = left, +1 = right.  speed: 0..100."""
    #keep rotating until the line is found again

    def _rotate_90_left(self): 
        line_l, line_r = self._sensors.line_trackers()
        self.in_action = 1

        while not (line_l and line_r): # 0/0, 0/1, 1/0 
            self._drive.rotate(-1, self._speed)

        self._drive.stop()
        self.in_action = 0
        return
    
    def _rotate_90_right(self):
        line_l, line_r = self._sensors.line_trackers()
        self.in_action = 1

        while not (line_l and line_r): # 0/0, 0/1, 1/0 
            self._drive.rotate(1, self._speed)

        self._drive.stop()
        self.in_action = 0
        return

    def _rotate_180(self): 
        # !!!!!!NOTe!!!!!! rotate_180 ONLY works while turning in the direction where theres a 90 turn path. 
        #only stop at the second successful line detection

        if 1:
            self._rotate_90_left()
            self._rotate_90_left()

        else:
            self._rotate_90_right()
            self._rotate_90_right()
    
        return
    # ---------------------------------------------------------- lost handling
    def _on_line(self):
        self._lost_since_ms = None

    def _line_lost(self):
        """Reverse to retrace onto the line, as per the flowchart.

        The flowchart reverses indefinitely.  That is right for the usual case
        - the AGV has overshot and backing up puts the line under the sensors
        again - but if the line is genuinely gone, reversing forever walks the
        AGV off the table.  So reversing is given a deadline: after
        lost_timeout_ms with no sight of the line at all, stop and stay
        stopped.  Set lost_timeout_ms=0 to reverse indefinitely instead.
        """
        now = ticks_ms()
        if self._lost_since_ms is None:
            self._lost_since_ms = now

        if (self._lost_timeout_ms and
                ticks_diff(now, self._lost_since_ms) >= self._lost_timeout_ms):
            self._drive.stop()
            return

        self._drive.move(-self._reverse_speed, 0)

    def stop(self):
        self._drive.stop()
        self._lost_since_ms = None



# 104,105,161 depends on map