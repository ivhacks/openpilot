import math

from openpilot.cereal import log

NAV_MAX_SPEED = 8.3                      # m/s
DESIRE_ON_BEARING = math.radians(15.)
DESIRE_OFF_BEARING = math.radians(10.)
# Enough to hold a line, far short of any real right turn (50 m radius).
MAX_RIGHT_CURVATURE = 0.02


class NavDesireInjector:
  """Maps the parkNavSignal (rotation toward a GPS destination) onto the model's
  trained left turn desire (log.Desire.turnLeft), reusing the same channel
  the blinker path uses. The physical blinker desire always takes precedence.

  Only left turns are requested. A destination to the right is left to the
  model's own path, so the car keeps going straight until the destination
  falls to its left.

  Turn activation/deactivation uses a hysteresis band around the relBearing
  threshold. relBearing is in the NED convention: positive = destination to the right.
  """

  def __init__(self):
    self.turn_active = False
    self.nav_active = False

  def update(self, nav_signal, nav_alive: bool, CS, lat_active: bool,
             v_ego: float, blinker_desire: log.Desire) -> log.Desire:
    if blinker_desire != log.Desire.none:
      self.turn_active = False
      self.nav_active = False
      return blinker_desire

    nav_ok = nav_alive and nav_signal.valid and lat_active and v_ego < NAV_MAX_SPEED
    self.nav_active = nav_ok
    if not nav_ok or nav_signal.relBearing > 0:
      self.turn_active = False
      return log.Desire.none

    left_bearing = -nav_signal.relBearing
    if not self.turn_active:
      if left_bearing > DESIRE_ON_BEARING:
        self.turn_active = True
      else:
        return log.Desire.none
    elif left_bearing < DESIRE_OFF_BEARING:
      self.turn_active = False
      return log.Desire.none
    # else: inside the hysteresis band, stay active

    if CS.leftBlindspot:
      return log.Desire.none
    return log.Desire.turnLeft
