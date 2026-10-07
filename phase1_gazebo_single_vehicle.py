#!/usr/bin/env python3
"""
Phase 1 of the hardware implementation roadmap (UAV_Swarm_Update_Claude_Code_Spec.md,
Appendix C): single-vehicle plumbing, validated in PX4 + Gazebo (not just headless SIH
SITL) so the vehicle dynamics and EKF behavior are closer to real hardware than the
earlier dynamic-election SITL smoke tests were. No swarm code here on purpose --
this is the foundation Phases 3/4 build on, validated in isolation first.

Validates exactly what Phase 1 in the roadmap calls for:
  1. arm -> takeoff -> offboard -> land lifecycle
  2. telemetry readback (position/velocity/battery)
  3. the setpoint-stream-lapse failure mode: what PX4 actually does when the
     offboard setpoint stream stops arriving (should fall back to hold/RTL,
     not crash or ignore it) -- the swarm daemon's later control loop has to
     guarantee a steady setpoint rate even while doing election/flocking math
     in the same loop, so it matters to see the real failure mode now rather
     than assume it.

Uses the body-velocity hold pattern (VelocityBodyYawspeed with clamped
proportional corrections) rather than raw PositionNedYaw for station-keeping,
per this project's own prior finding that a pure position setpoint can produce
a large runaway under EKF load-induced drift -- safer default for a plumbing
test that has no reason to need absolute position control yet.
"""

import asyncio
import math
import os
import sys
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

GRPC_PORT = 50051


def wrap_deg(a):
    return (a + 180) % 360 - 180


ALT_HOLD_KP, ALT_HOLD_MAX = 0.4, 1.0
YAW_HOLD_KP, YAW_HOLD_MAX = 1.0, 10.0


async def wait_for_load_to_settle(max_wait_s=45):
    target = 1.6 * (os.cpu_count() or 1)
    t0 = time.monotonic()
    while time.monotonic() - t0 < max_wait_s:
        load1, _, _ = os.getloadavg()
        if load1 < target:
            print(f"[phase1] system load settled ({load1:.1f}, threshold {target:.1f})")
            return
        await asyncio.sleep(2)
    print("[phase1] proceeding despite high load -- watch for pre-arm failsafes")


async def wait_ekf_ready(drone, timeout=60.0):
    t0 = asyncio.get_event_loop().time()
    async for h in drone.telemetry.health():
        print(f"[phase1] health: global_pos_ok={h.is_global_position_ok} "
              f"home_pos_ok={h.is_home_position_ok} armable={h.is_armable}")
        if h.is_global_position_ok and h.is_home_position_ok:
            return
        if asyncio.get_event_loop().time() - t0 > timeout:
            raise TimeoutError("EKF/home position never became ready")


async def arm_with_retries(drone, attempts=15, delay_s=3.0):
    for i in range(attempts):
        try:
            await drone.action.arm()
            print(f"[phase1] armed (attempt {i + 1})")
            return
        except Exception as e:
            print(f"[phase1] arm attempt {i + 1}/{attempts} failed ({e}), retrying...")
            await asyncio.sleep(delay_s)
    raise RuntimeError("Could not arm after retries")


async def print_telemetry_once(drone):
    pvn = None
    async for p in drone.telemetry.position_velocity_ned():
        pvn = p
        break
    batt = None
    async for b in drone.telemetry.battery():
        batt = b
        break
    print(f"[telemetry] pos(n,e,d)=({pvn.position.north_m:.2f},{pvn.position.east_m:.2f},"
          f"{pvn.position.down_m:.2f})  vel(n,e,d)=({pvn.velocity.north_m_s:.2f},"
          f"{pvn.velocity.east_m_s:.2f},{pvn.velocity.down_m_s:.2f})  "
          f"battery={batt.remaining_percent:.0f}%")  # already 0-100, not a 0-1 fraction
    return pvn, batt


async def main():
    await wait_for_load_to_settle()

    print(f"[phase1] connecting on mavsdk_server :{GRPC_PORT} ...")
    drone = System(mavsdk_server_address="127.0.0.1", port=GRPC_PORT)
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    print("[phase1] mavsdk connected")

    await wait_ekf_ready(drone)

    already_airborne = False
    async for in_air in drone.telemetry.in_air():
        already_airborne = in_air
        break

    if already_airborne:
        # The print used to say "skipping takeoff" but the code fell through to
        # arm_with_retries() + takeoff() anyway -- rerunning this script against
        # an already-airborne vehicle could then fail on re-arming or issue a
        # second takeoff command (found via Sourcery review).
        print("[phase1] already in air, skipping takeoff")
    else:
        await arm_with_retries(drone)
        await drone.action.set_takeoff_altitude(5.0)
        await drone.action.takeoff()
        print("[phase1] takeoff commanded, waiting to reach altitude...")
        for _ in range(60):
            pvn = None
            async for p in drone.telemetry.position_velocity_ned():
                pvn = p
                break
            alt = -pvn.position.down_m
            if alt > 4.0:
                print(f"[phase1] reached altitude {alt:.2f}m")
                break
            await asyncio.sleep(0.5)

    # --- 1. telemetry readback ---
    print("\n=== 1. TELEMETRY READBACK ===")
    pvn0, batt0 = await print_telemetry_once(drone)
    target_alt = -pvn0.position.down_m
    target_yaw = 0.0
    async for att in drone.telemetry.attitude_euler():
        target_yaw = att.yaw_deg
        break

    # --- 2. enter offboard with a hold setpoint, confirm it holds ---
    print("\n=== 2. OFFBOARD HOLD (body-velocity, clamped corrections) ===")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[phase1] offboard start FAILED: {e}")
        sys.exit(1)
    print("[phase1] offboard started")

    hold_task_stop = False

    async def hold_loop():
        while not hold_task_stop:
            pvn = None
            async for p in drone.telemetry.position_velocity_ned():
                pvn = p
                break
            yaw = target_yaw
            async for att in drone.telemetry.attitude_euler():
                yaw = att.yaw_deg
                break
            current_alt = -pvn.position.down_m
            down_corr = max(-ALT_HOLD_MAX, min(ALT_HOLD_MAX, -ALT_HOLD_KP * (target_alt - current_alt)))
            yaw_err = wrap_deg(target_yaw - yaw)
            yaw_corr = max(-YAW_HOLD_MAX, min(YAW_HOLD_MAX, YAW_HOLD_KP * yaw_err))
            try:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, down_corr, yaw_corr))
            except OffboardError as e:
                print(f"[phase1] hold setpoint error: {e}")
            await asyncio.sleep(0.05)  # 20 Hz

    ht = asyncio.create_task(hold_loop())
    await asyncio.sleep(8.0)
    print("[phase1] held for 8s, checking altitude stayed near target:")
    await print_telemetry_once(drone)

    # --- 3. setpoint-stream-lapse failure mode ---
    print("\n=== 3. SETPOINT-STREAM-LAPSE TEST ===")
    print("[phase1] stopping the setpoint stream for 6s to observe PX4's fallback behavior...")
    hold_task_stop = True
    await ht
    t_lapse_start = time.monotonic()
    for i in range(6):
        await asyncio.sleep(1.0)
        fm = "?"
        async for m in drone.telemetry.flight_mode():
            fm = str(m)
            break
        print(f"[phase1] t+{time.monotonic() - t_lapse_start:.1f}s  flight_mode={fm}")

    print("[phase1] resuming setpoint stream...")
    hold_task_stop = False
    ht2 = asyncio.create_task(hold_loop())
    await asyncio.sleep(5.0)
    fm = "?"
    async for m in drone.telemetry.flight_mode():
        fm = str(m)
        break
    print(f"[phase1] after resume, flight_mode={fm}")
    if fm == "OFFBOARD":
        print("[phase1] recovered into OFFBOARD cleanly after the lapse")
    else:
        print(f"[phase1] NOTE: not back in OFFBOARD automatically (mode={fm}) -- "
              f"the real controller will need to re-call offboard.start() after a lapse "
              f"like this, not just resume streaming setpoints")
        try:
            await drone.offboard.start()
            print("[phase1] re-entered OFFBOARD explicitly")
        except OffboardError as e:
            print(f"[phase1] could not re-enter OFFBOARD: {e}")

    hold_task_stop = True
    await ht2

    # --- 4. land ---
    print("\n=== 4. LAND ===")
    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    await drone.action.land()
    for _ in range(60):
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                print("[phase1] landed")
                return
            break
        await asyncio.sleep(1.0)
    print("[phase1] land timed out waiting for in_air=False (check manually)")


if __name__ == "__main__":
    asyncio.run(main())
