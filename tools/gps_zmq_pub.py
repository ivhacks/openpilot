#!/usr/bin/env python3
"""GPS/desire ZMQ publisher: runs on the comma device, reads live position and
lateral-desire state off the existing openpilot message bus, and republishes
compact JSON summaries over a plain ZMQ PUB socket so a laptop can subscribe
(see gps_rerun_viewer.py / zmq_debug_viewer.py) without needing cereal/capnp
installed locally. Read-only: does not touch the GNSS receiver, the car, or
any openpilot process -- it only subscribes to messages already flowing, and
runs the *actual* on-device DesireHelper/NavDesireInjector classes against
that bus to reconstruct the desire modeld computes (which is itself never
published to any log/topic).

Topics published: "gps" (lat/lon position only), "desire" (the raw
log.Desire ordinal as a float, no sign/name interpretation -- only the
blinker booleans are collapsed into a left/right/none state), and "parknav"
(raw parkNavSignal fields plus the "ParkingDestination" param's lat/lon --
alive/valid even when parknavd isn't running, so you can see it's dead
rather than guessing).

Run on the device (this kills any previous instance already bound to the
port so you can just re-run this after editing without a manual pkill):

    cd /data/openpilot
    PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 tools/gps_zmq_pub.py
"""
import json
import os
import signal
import time

import zmq

from openpilot.cereal.messaging import SubMaster
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.modeld.park_nav import NavDesireInjector

PORT = 5555
BIND_RETRY_S = 3.0


def get_destination(params: Params) -> tuple[float, float] | None:
  """Mirrors system/parknav/parknavd.py's own get_destination() exactly."""
  dest = params.get("ParkingDestination")
  if dest is None:
    return None
  try:
    return float(dest["latitude"]), float(dest["longitude"])
  except Exception:
    return None


def kill_listener_on_port(port: int) -> None:
  """Best-effort: SIGTERM whatever process is already listening on `port` (Linux /proc only),
  so re-running this script doesn't require a manual pkill of the previous instance first."""
  target_hex = format(port, "04X")
  try:
    with open("/proc/net/tcp") as f:
      lines = f.readlines()[1:]
  except OSError:
    return

  inodes = set()
  for line in lines:
    fields = line.split()
    local_port = fields[1].split(":")[1]
    state = fields[3]
    if local_port.upper() == target_hex and state == "0A":  # 0A = LISTEN
      inodes.add(fields[9])
  if not inodes:
    return

  my_pid = os.getpid()
  for pid_str in os.listdir("/proc"):
    if not pid_str.isdigit() or int(pid_str) == my_pid:
      continue
    fd_dir = f"/proc/{pid_str}/fd"
    try:
      fds = os.listdir(fd_dir)
    except OSError:
      continue
    for fd in fds:
      try:
        link = os.readlink(f"{fd_dir}/{fd}")
      except OSError:
        continue
      if link.startswith("socket:[") and link[8:-1] in inodes:
        pid = int(pid_str)
        print(f"gps_zmq_pub: killing existing process {pid} listening on port {port}", flush=True)
        try:
          os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
          pass
        break


def bind_pub_socket(ctx: zmq.Context, port: int) -> zmq.Socket:
  kill_listener_on_port(port)
  sock = ctx.socket(zmq.PUB)
  deadline = time.monotonic() + BIND_RETRY_S
  while True:
    try:
      sock.bind(f"tcp://*:{port}")
      return sock
    except zmq.error.ZMQError:
      if time.monotonic() > deadline:
        raise
      time.sleep(0.2)


def main():
  sm = SubMaster(
    ["gpsLocationExternal", "carState", "carControl", "parkNavSignal"],
    poll="gpsLocationExternal",
  )
  params = Params()
  desire_helper = DesireHelper()
  nav_injector = NavDesireInjector()

  ctx = zmq.Context()
  sock = bind_pub_socket(ctx, PORT)

  print(f"gps_zmq_pub: publishing on tcp://*:{PORT} topics=gps,desire,parknav", flush=True)

  while True:
    sm.update(1000)

    if sm.seen["carState"] and sm.seen["carControl"]:
      cs = sm["carState"]
      cc = sm["carControl"]
      nav_signal = sm["parkNavSignal"]
      nav_alive = sm.alive["parkNavSignal"]

      desire_helper.update(cs, cc.latActive)
      final_desire = float(nav_injector.update(nav_signal, nav_alive, cs, cc.latActive, cs.vEgo, desire_helper.desire))
      blinker_desire = float(desire_helper.desire)
      # The one allowed interpretation: collapse the raw blinker booleans into a left/right/none state.
      blinker = -1 if (cs.leftBlinker and not cs.rightBlinker) else (1 if (cs.rightBlinker and not cs.leftBlinker) else 0)

      t = time.time()
      desire_msg = {
        "t": t,
        "desire": final_desire,
        "blinkerDesire": blinker_desire,
        "blinkerSigned": blinker,
        "leftBlinker": bool(cs.leftBlinker),
        "rightBlinker": bool(cs.rightBlinker),
        "leftBlindspot": bool(cs.leftBlindspot),
        "rightBlindspot": bool(cs.rightBlindspot),
        "latActive": bool(cc.latActive),
        "vEgo": float(cs.vEgo),
      }
      sock.send_multipart([b"desire", json.dumps(desire_msg).encode()])

      destination = get_destination(params)
      parknav_msg = {
        "t": t,
        "alive": bool(nav_alive),
        "valid": bool(nav_signal.valid) if sm.seen["parkNavSignal"] else False,
        "relBearing": float(nav_signal.relBearing) if sm.seen["parkNavSignal"] else None,
        "lateralOffset": float(nav_signal.lateralOffset) if sm.seen["parkNavSignal"] else None,
        "forwardDist": float(nav_signal.forwardDist) if sm.seen["parkNavSignal"] else None,
        "totalDist": float(nav_signal.totalDist) if sm.seen["parkNavSignal"] else None,
        "destLatitude": destination[0] if destination is not None else None,
        "destLongitude": destination[1] if destination is not None else None,
      }
      sock.send_multipart([b"parknav", json.dumps(parknav_msg).encode()])

    if not sm.updated["gpsLocationExternal"]:
      continue

    gps = sm["gpsLocationExternal"]
    msg = {
      "t": time.time(),
      "latitude": gps.latitude,
      "longitude": gps.longitude,
    }
    sock.send_multipart([b"gps", json.dumps(msg).encode()])


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
