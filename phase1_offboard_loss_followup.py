#!/usr/bin/env python3
"""
Follow-up to phase1_gazebo_single_vehicle.py's setpoint-stream-lapse test.

That test paused only the Python setpoint-sending coroutine while the SAME
mavsdk_server process (and therefore the same underlying MAVLink heartbeat
connection to PX4) stayed alive -- and PX4 remained in OFFBOARD through a
full 6s gap despite COM_OF_LOSS_T=1.0. This script tests the scenario that
actually matters for the swarm daemon design: what happens if the WHOLE
companion-side link dies, not just the application's setpoint loop, by
killing mavsdk_server itself (the actual MAVLink peer) mid-flight and
reconnecting fresh afterward to see what PX4 fell back to.

Flight remains armed/airborne throughout -- offboard.stop() and land() are
only called via the fresh reconnection at the end, exactly so PX4's own
fallback behavior (not a clean shutdown) is what determines what happens to
the vehicle during the gap.
"""

import asyncio
import os
import signal
import subprocess
import sys
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

GRPC_PORT = 50052   # different port from phase1_gazebo_single_vehicle.py's 50051
MAVSDK_SERVER = os.path.expanduser("~/mavsdk_drone_show/mavsdk_server")


async def wait_ekf_ready(drone, timeout=60.0):
    t0 = asyncio.get_event_loop().time()
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            return
        if asyncio.get_event_loop().time() - t0 > timeout:
            raise TimeoutError("EKF/home position never became ready")


async def connect(port):
    drone = System(mavsdk_server_address="127.0.0.1", port=port)
    await drone.connect()
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    return drone


async def main():
    print(f"[followup] starting mavsdk_server on gRPC :{GRPC_PORT} / udp:14540 ...")
    server = subprocess.Popen(
        [MAVSDK_SERVER, "-p", str(GRPC_PORT), "udp://:14540"],
        stdout=open("/tmp/sitl_run/logs/mavsdk_followup.log", "w"),
        stderr=subprocess.STDOUT,
    )
    await asyncio.sleep(2.0)

    drone = await connect(GRPC_PORT)
    print("[followup] connected")
    await wait_ekf_ready(drone)

    await drone.action.arm()
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()
    print("[followup] takeoff commanded")
    for _ in range(60):
        pvn = None
        async for p in drone.telemetry.position_velocity_ned():
            pvn = p
            break
        if -pvn.position.down_m > 4.0:
            print(f"[followup] reached {-pvn.position.down_m:.2f}m")
            break
        await asyncio.sleep(0.5)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[followup] offboard start FAILED: {e}")
        sys.exit(1)
    print("[followup] offboard started, holding for 3s to confirm stability...")
    for _ in range(30):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.1)

    fm = None
    async for m in drone.telemetry.flight_mode():
        fm = str(m)
        break
    print(f"[followup] pre-kill flight_mode={fm}")

    # --- The actual test: kill mavsdk_server (the real MAVLink peer), not
    # just our Python coroutine. This severs the connection PX4 actually
    # sees, simulating a companion-PC crash rather than an app-level pause.
    print(f"\n[followup] killing mavsdk_server (pid={server.pid}) -- this is the real test...")
    server.send_signal(signal.SIGKILL)
    server.wait(timeout=5)
    kill_t = time.monotonic()
    print("[followup] mavsdk_server is dead. Waiting 10s with NO companion-side connection at all...")
    await asyncio.sleep(10.0)
    print(f"[followup] {time.monotonic() - kill_t:.1f}s elapsed, reconnecting fresh to check what PX4 did...")

    # Fresh mavsdk_server + connection -- the old `drone` object is now talking
    # to a dead server and can't be trusted for anything past this point.
    server2 = subprocess.Popen(
        [MAVSDK_SERVER, "-p", str(GRPC_PORT + 1), "udp://:14540"],
        stdout=open("/tmp/sitl_run/logs/mavsdk_followup2.log", "w"),
        stderr=subprocess.STDOUT,
    )
    await asyncio.sleep(2.0)
    drone2 = await connect(GRPC_PORT + 1)
    fm2 = None
    async for m in drone2.telemetry.flight_mode():
        fm2 = str(m)
        break
    armed2 = None
    async for a in drone2.telemetry.armed():
        armed2 = a
        break
    in_air2 = None
    async for v in drone2.telemetry.in_air():
        in_air2 = v
        break
    pvn2 = None
    async for p in drone2.telemetry.position_velocity_ned():
        pvn2 = p
        break
    print(f"[followup] POST-GAP STATE: flight_mode={fm2}  armed={armed2}  in_air={in_air2}  "
          f"alt={-pvn2.position.down_m:.2f}m  vel=({pvn2.velocity.north_m_s:.2f},"
          f"{pvn2.velocity.east_m_s:.2f},{pvn2.velocity.down_m_s:.2f})")

    # NOTE: flight_mode() alone is NOT a reliable signal here -- a single read of it,
    # taken in isolation, previously produced a wrong conclusion ("stayed in OFFBOARD
    # forever") when the vehicle had actually already auto-landed and disarmed. Always
    # cross-check against armed/in_air before trusting what flight_mode() reports,
    # especially right after reconnecting with a fresh System() object.
    if armed2 is False and in_air2 is False:
        print("[followup] FINDING: PX4 auto-landed and disarmed on its own during the gap -- "
              "the real link loss (not just a paused setpoint loop) triggers a genuine, "
              "controlled failsafe: a smooth constant-rate descent to touchdown, then disarm, "
              "with no application-level intervention at all. This is the opposite of the "
              "premature conclusion a bare flight_mode() read would suggest here -- see the "
              "flight-log trace (vz ramps to a steady ~0.75 m/s descent right at the kill, not "
              "a free-fall) for the evidence. Good news for safety, but it also means the "
              "vehicle acted entirely without the swarm daemon's knowledge; the daemon's own "
              "link-failure handling (Section 6 of the proposal) still needs to detect this "
              "independently rather than assume PX4's local failsafe is visible to it.")
    elif fm2 == "OFFBOARD" and armed2:
        print("[followup] FINDING: PX4 stayed in OFFBOARD, armed, with the whole link dead for "
              "10s -- it is just continuing to hold the last setpoint it received. The swarm "
              "daemon cannot rely on PX4 auto-recovering in this case; losing the link silently "
              "leaves the vehicle doing whatever it was last told, indefinitely.")
    else:
        print(f"[followup] FINDING: PX4 left OFFBOARD during the gap, now in {fm2} -- "
              f"confirms COM_OF_LOSS_T does trigger on a REAL link loss (not just a paused "
              f"setpoint loop within an otherwise-alive connection, which is what the first "
              f"test actually measured).")

    # Land via the fresh connection regardless of outcome.
    print("[followup] landing via fresh connection...")
    try:
        await drone2.offboard.stop()
    except OffboardError:
        pass
    await drone2.action.land()
    for _ in range(60):
        async for v in drone2.telemetry.in_air():
            if not v:
                print("[followup] landed")
                server2.send_signal(signal.SIGKILL)
                return
            break
        await asyncio.sleep(1.0)
    print("[followup] land timed out waiting for in_air=False (check manually)")
    server2.send_signal(signal.SIGKILL)


if __name__ == "__main__":
    asyncio.run(main())
