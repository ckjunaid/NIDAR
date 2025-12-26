#!/usr/bin/env python3
import asyncio
import time
import sys
import termios
import tty

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
import aioconsole


MAVSDK_CONNECTION = "serial:///dev/ttyUSB0:57600"

CRUISE_ALT = 10
HOLD_DESCEND_ALT = 5.0
HOLD_TIME = 10
HOLD_DESCENT_SPEED = 0.7
HOLD_ASCENT_SPEED = 0.8

MAX_HOLDS_BEFORE_RTL = 5


async def get_current_altitude(drone: System) -> float:
    async for p in drone.telemetry.position():
        return p.relative_altitude_m
    return 0.0


async def go_to_altitude(drone: System, target_alt: float, speed: float = 0.7, tolerance: float = 0.3):
    print(f"[ALT] Target altitude: {target_alt:.2f} m")

    while True:
        alt = await get_current_altitude(drone)
        err = target_alt - alt

        if abs(err) <= tolerance:
            break

        vz_cmd = -speed if err > 0 else speed
        vz_cmd = max(min(vz_cmd, 1.5), -1.5)

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, vz_cmd, 0.0)
        )
        await asyncio.sleep(0.25)

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(0.3)


async def descend_hold_ascend(
    drone: System,
    descend_alt: float = HOLD_DESCEND_ALT,
    hold_time: int = HOLD_TIME,
    is_final_hold: bool = False,
):
    if is_final_hold:
        print("[FINAL] 5th hold reached — executing RTL")
        try:
            await drone.mission.pause_mission()
        except Exception:
            pass

        try:
            await drone.action.return_to_launch()
            print("[FINAL] RTL commanded")
        except Exception as e:
            print(f"[FINAL] RTL failed: {e}")
            return False

        async for landed in drone.telemetry.landed_state():
            if landed == drone.telemetry.LandedState.ON_GROUND:
                print("[FINAL] Landed successfully")
                return True
            await asyncio.sleep(1)

        return True

    try:
        await drone.mission.pause_mission()
    except Exception as e:
        print(f"[HOLD] pause_mission warning: {e}")

    try:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[HOLD] Offboard start failed: {e}")
        return False

    try:
        await go_to_altitude(drone, descend_alt, HOLD_DESCENT_SPEED)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        for i in range(hold_time):
            print(f"[HOLD] Holding {i + 1}/{hold_time}")
            await asyncio.sleep(1)

        await go_to_altitude(drone, CRUISE_ALT, HOLD_ASCENT_SPEED)

    except Exception as e:
        print(f"[HOLD] Hold sequence error: {e}")
        return False

    try:
        await drone.offboard.stop()
    except Exception:
        pass

    try:
        await drone.mission.start_mission()
    except Exception:
        pass

    print("[HOLD] Hold sequence complete\n" + "=" * 60)
    return True


async def mission_command_loop(drone: System):
    print("[INFO] Command loop started (h=hold, q=quit)")
    holds = 0

    while True:
        cmd = (await aioconsole.ainput("> ")).strip().lower()

        if cmd == "h":
            holds += 1
            is_final = holds >= MAX_HOLDS_BEFORE_RTL

            success = await descend_hold_ascend(drone, is_final_hold=is_final)

            if is_final and success:
                print("[INFO] RTL completed — exiting command loop")
                break

        elif cmd == "q":
            print("[INFO] Landing requested")
            try:
                await drone.action.land()
            except Exception:
                pass

            async for ls in drone.telemetry.landed_state():
                if ls == drone.telemetry.LandedState.ON_GROUND:
                    break
                await asyncio.sleep(1)

            break

        else:
            print("[WARN] Unknown command")


async def arm_and_takeoff_to_cruise(drone: System):
    try:
        await drone.action.set_takeoff_altitude(CRUISE_ALT)
        await drone.action.arm()
        await drone.action.takeoff()
    except Exception:
        pass

    start = time.time()
    while time.time() - start < 30:
        alt = await get_current_altitude(drone)
        if alt >= CRUISE_ALT - 1:
            break
        await asyncio.sleep(0.5)

    try:
        await drone.mission.start_mission()
    except Exception:
        pass


async def main():
    drone = System()
    await drone.connect(system_address=MAVSDK_CONNECTION)

    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    await arm_and_takeoff_to_cruise(drone)
    await mission_command_loop(drone)

    try:
        await drone.offboard.stop()
    except Exception:
        pass


if __name__ == "__main__":
    asyncio.run(main())

