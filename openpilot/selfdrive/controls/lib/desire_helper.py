from openpilot.cereal import log

class DesireHelper:
  def __init__(self):
    self.lane_change_state = log.LaneChangeState.off
    self.lane_change_direction = log.LaneChangeDirection.none
    self.desire = log.Desire.none
    self.turn_pulsed = False

  def update(self, carstate, lateral_active):
    if not lateral_active or not carstate.leftBlinker or carstate.rightBlinker:
      self.desire = log.Desire.none
      self.turn_pulsed = False
      return

    if self.turn_pulsed:
      self.desire = log.Desire.turnLeft
      return

    if carstate.leftBlindspot:
      self.desire = log.Desire.none
      return

    self.desire = log.Desire.turnLeft
    self.turn_pulsed = True
