#!/usr/bin/env python3
import asyncio
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import LandedState
import aioconsole


MAVSDK_CONNECTION = "serial:///dev/ttyUSB0:57600"
CRUISE_ALT = 10.0
HOLD_DESCEND_ALT = 5.0
HOLD_TIME = 10
HOLD_DESCENT_SPEED = 0.7
HOLD_ASCENT_SPEED = 0.8
MAX_HOLDS_BEFORE_RTL = 5


async def get_current_altitude(drone: System) -> float:
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m
    return 0.0

async def stream_zero_velocity(drone: System, duration_s: float):
    """PX4 Offboard requires continuous setpoints."""
    interval = 0.05
    count = int(duration_s / interval)
    for _ in range(count):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(interval)

async def go_to_altitude(
    drone: System,
    target_alt: float,
    speed: float,
    tolerance: float = 0.3,
    timeout: float = 20.0,
):
    print(f"[ALT] Target altitude: {target_alt:.2f} m")
    start = time.time()
    while True:
        if time.time() - start > timeout:
            raise TimeoutError("Altitude change timeout")
        alt = await get_current_altitude(drone)
        error = target_alt - alt
        if abs(error) <= tolerance:
            break
        vz = -speed if error > 0 else speed  # negative vz to climb
        vz = max(min(vz, 1.5), -1.5)
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, vz, 0.0)
        )
        await asyncio.sleep(0.25)
    await stream_zero_velocity(drone, 0.3)


async def descend_hold_ascend(
    drone: System,
    descend_alt: float = HOLD_DESCEND_ALT,
    hold_time: int = HOLD_TIME,
):
    try:
        await drone.mission.pause_mission()
        print("[PAUSE] Mission paused successfully")
    except Exception as e:
        print(f"[PAUSE] pause_mission failed: {e}")

    await stream_zero_velocity(drone, 1.0)

    try:
        await drone.offboard.start()
        print("[OFFBOARD] Started successfully")
    except OffboardError as e:
        print(f"[OFFBOARD] Start failed: {e}")
        return False

    try:
        await go_to_altitude(drone, descend_alt, HOLD_DESCENT_SPEED)

        for i in range(hold_time):
            print(f"[HOLD] Holding {i + 1}/{hold_time}")
            await stream_zero_velocity(drone, 1.0)

        await go_to_altitude(drone, CRUISE_ALT, HOLD_ASCENT_SPEED)

        print("[TRANSITION] Final zero velocity stream for smooth exit...")
        await stream_zero_velocity(drone, 5.0)

    except Exception as e:
        print(f"[HOLD] Error during hold sequence: {e}")
        return False
    finally:
        try:
            await drone.action.set_flight_mode("AUTO")
            print("[TRANSITION] Requested flight mode: AUTO - mission should resume")
        except Exception as e:
            print(f"[TRANSITION] offboard.stop failed: {e}")

        await asyncio.sleep(1.0)

        try:
            await drone.mission.start_mission()
            print("[TRANSITION] start_mission commanded → mission resuming")
        except Exception as e:
            print(f"[TRANSITION] start_mission failed: {e}")

    print("[HOLD] Hold sequence complete – mission resumed")
    print("=" * 60)
    return True

async def mission_command_loop(drone: System):
    print("[INFO] Ready for commands")
    print("[INFO] h = hold | q = land")
    holds = 0
    while True:
        cmd = (await aioconsole.ainput("> ")).strip().lower()
        if cmd == "h":
            holds += 1
            print(f"[INFO] Hold {holds}/{MAX_HOLDS_BEFORE_RTL}")
            success = await descend_hold_ascend(drone)
            if holds >= MAX_HOLDS_BEFORE_RTL and success:
                print("[FINAL] 5th hold completed")
                print("[FINAL] Waiting 10 seconds at mission altitude")
                for i in range(2):
                    print(f"[FINAL] Wait {i + 1}/10")
                    await asyncio.sleep(1)
                try:
                    await drone.mission.pause_mission()
                    await drone.action.return_to_launch()
                    print("[FINAL] RTL commanded")
                except Exception as e:
                    print(f"[FINAL] RTL failed: {e}")
                    break

                start = time.time()
                async for landed in drone.telemetry.landed_state():
                    if landed == LandedState.ON_GROUND:
                        print("[FINAL] Landed successfully")
                        break
                    if time.time() - start > 120:
                        print("[FINAL] RTL timeout")
                        break
                    await asyncio.sleep(1)
                print("[FINAL] Mission complete")
                break

        elif cmd == "q":
            print("[INFO] Landing requested")
            try:
                await drone.action.land()
            except Exception:
                pass
            async for landed in drone.telemetry.landed_state():
                if landed == LandedState.ON_GROUND:
                    break
                await asyncio.sleep(1)
            break
        else:
            print("[WARN] Unknown command")

async def main():
    drone = System()
    print(f"[DRONE] Connecting to {MAVSDK_CONNECTION}")
    await drone.connect(system_address=MAVSDK_CONNECTION)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[DRONE] Connected")
            break

    print("[INFO] Mission NOT auto-started")
    print("[INFO] Start mission manually from QGC (switch to AUTO mode)")
    print("[INFO] Mission will run, press 'h' during flight to trigger holds")

    await mission_command_loop(drone)

    print("[MAIN] Script finished")

if __name__ == "__main__":
    asyncio.run(main())
