
#!/usr/bin/env python3
import asyncio
import time
import sys
import termios
import tty

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
import aioconsole


MAVSDK_CONNECTION = "serial:///dev/ttyACM0:57600"


CRUISE_ALT = 20.0       
HOLD_DESCEND_ALT = 5.0 
HOLD_TIME = 10          
HOLD_DESCENT_SPEED = 0.7
HOLD_ASCENT_SPEED = 0.8

MAX_HOLDS_BEFORE_RTL = 5


async def get_current_altitude(drone: System) -> float:
    """Return current relative altitude (meters)."""
    async for p in drone.telemetry.position():
        return p.relative_altitude_m
    return 0.0

async def go_to_altitude(drone: System, target_alt: float, speed: float = 0.7, tolerance: float = 0.3):

    curr_alt = await get_current_altitude(drone)
    print(f"[ALT] Current: {curr_alt:.2f} m -> Target: {target_alt:.2f} m")

    while True:
        alt = await get_current_altitude(drone)
        err = target_alt - alt
        if abs(err) <= tolerance:
            print(f"[ALT] Reached {alt:.2f} m (within {tolerance} m)")
            break

        vz_cmd = -speed if err > 0 else speed
        # clamp
        vz_cmd = max(min(vz_cmd, 1.5), -1.5)
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, vz_cmd, 0.0))
        await asyncio.sleep(0.25)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(0.3)

async def descend_hold_ascend(drone: System, descend_alt: float = HOLD_DESCEND_ALT, hold_time: int = HOLD_TIME, is_final_hold: bool = False):
    """
    Pause mission, enter offboard, descend to descend_alt, hold for hold_time seconds,
    ascend back to CRUISE_ALT, stop offboard, and resume mission.
    Returns True on success, False on any failure.
    """
    if is_final_hold:
        print(f"[FINAL] 5th hold complete and back at cruise altitude. Executing RTL...")
        try:
            
            try:
                await drone.mission.pause_mission()
            except Exception:
                pass
            await drone.action.return_to_launch()
            print("[FINAL] Return to launch commanded.")
    
            async for landed in drone.telemetry.landed_state():
                if landed == drone.telemetry.LandedState.ON_GROUND:
                    print("[FINAL] Drone landed successfully.")
                    return True
                await asyncio.sleep(1)
            return True
        except Exception as e:
            print(f"[FINAL] RTL failed: {e}")
            return False

    print("[HOLD] Hold sequence complete.")
    print("="*60 + "\n")
    return True
    try:
        print("[HOLD] Pausing mission...")
        await drone.mission.pause_mission()
    except Exception as e:
        print(f"[HOLD] Warning: pause_mission failed: {e} (continuing)")

    try:
        print("[HOLD] Preparing offboard (zero velocity) and starting offboard...")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.start()
        print("[HOLD] Offboard started.")
    except OffboardError as oe:
        print(f"[HOLD] Offboard start failed: {oe}")
        return False
    except Exception as e:
        print(f"[HOLD] Offboard start unknown error: {e}")
        return False

   
    try:
        print(f"[HOLD] Descending to {descend_alt} m...")
        await go_to_altitude(drone, descend_alt, speed=HOLD_DESCENT_SPEED)
    except Exception as e:
        print(f"[HOLD] Descent failed: {e}")
        try:
            await drone.offboard.stop()
            await drone.mission.start_mission()
        except Exception:
            pass
        return False

    try:
        print(f"[HOLD] Holding at {descend_alt} m for {hold_time} seconds...")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        for i in range(hold_time):
            print(f"[HOLD] Hold: {i+1}/{hold_time} s")
            await asyncio.sleep(1.0)
    except Exception as e:
        print(f"[HOLD] Hold failed: {e}")
        try:
            await drone.offboard.stop()
            await drone.mission.start_mission()
        except Exception:
            pass
        return False

 
    try:
        print(f"[HOLD] Ascending back to cruise altitude: {CRUISE_ALT} m...")
        await go_to_altitude(drone, CRUISE_ALT, speed=HOLD_ASCENT_SPEED)
    except Exception as e:
        print(f"[HOLD] Ascent failed: {e}")
        try:
            await drone.offboard.stop()
            await drone.mission.start_mission()
        except Exception:
            pass
        return False

    try:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.stop()
        print("[HOLD] Offboard stopped.")
    except Exception as e:
        print(f"[HOLD] Offboard stop warning: {e}")

    try:
        await drone.mission.start_mission()
        print("[HOLD] Mission resumed.")
    except Exception as e:
        print(f"[HOLD] Mission resume warning: {e}")

    print("[HOLD] Hold sequence complete.")
    print("="*60 + "\n")
    return True


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


async def mission_command_loop(drone: System):
    """
    Listen for ground commands (via console). Each 'h' triggers the descend/hold/ascend sequence.
    After MAX_HOLDS_BEFORE_RTL holds, command the drone to return to launch (base).
    'q' lands the drone and exits.
    """
    print(f"[INFO] Command loop started. Type 'h' + Enter to trigger hold, 'q' + Enter to quit.")
    holds = 0

    while True:
        cmd = await aioconsole.ainput("Command (h=hold, q=quit): ")
        cmd = cmd.strip().lower()
        if cmd == "h":
            holds += 1
            print(f"[INFO] Hold command received ({holds}/{MAX_HOLDS_BEFORE_RTL})")
            is_final = (holds >= MAX_HOLDS_BEFORE_RTL)
            success = await descend_hold_ascend(drone, is_final_hold=is_final)
            if not success:
                print("[WARN] Hold sequence failed; continuing command loop.")
            if is_final and success:
                print("[INFO] RTL completed after 5th hold. Exiting command loop.")
                break
                    # stop mission (if running) and return to launch
                    try:
                        await drone.mission.pause_mission()
                    except Exception:
                        pass
                    await drone.action.return_to_launch()
                    print("[INFO] Return to launch commanded.")
                except Exception as e:
                    print(f"[ERROR] RTL failed: {e}")
                # wait until landed
                async for landed in drone.telemetry.landed_state():
                    if landed == drone.telemetry.LandedState.ON_GROUND:
                        break
                    await asyncio.sleep(1)
                print("[INFO] Drone reports landed. Exiting.")
                break

        elif cmd == "q":
            print("[INFO] Quit requested. Attempting to land...")
            try:
                await drone.action.land()
            except Exception as e:
                print(f"[WARN] Land command failed: {e}")
            # wait until landed
            async for ls in drone.telemetry.landed_state():
                if ls == drone.telemetry.LandedState.ON_GROUND:
                    break
                await asyncio.sleep(1)
            print("[INFO] Landed. Exiting.")
            break
        else:
            print("[WARN] Unknown command. Use 'h' or 'q'.")

async def arm_and_takeoff_to_cruise(drone: System):
    """Arms and takeoff to CRUISE_ALT (best-effort)."""
    print("[BOOT] Arming and taking off to cruise altitude...")
    try:
        await drone.action.set_takeoff_altitude(CRUISE_ALT)
    except Exception:
        pass

    try:
        await drone.action.arm()
        await asyncio.sleep(1.0)
    except Exception as e:
        print(f"[BOOT] Arm failed or already armed: {e}")

    try:
        await drone.action.takeoff()
    except Exception as e:
        print(f"[BOOT] Takeoff command failed: {e}")

    timeout = 30
    start_t = time.time()
    while True:
        alt = await get_current_altitude(drone)
        if alt >= CRUISE_ALT - 1.0:
            print(f"[BOOT] Reached cruise altitude: {alt:.1f} m")
            break
        if time.time() - start_t > timeout:
            print("[BOOT] Timeout waiting for cruise altitude; proceeding anyway.")
            break
        await asyncio.sleep(0.5)


    try:
        await drone.mission.start_mission()
        print("[BOOT] Mission started.")
    except Exception as e:
        print(f"[BOOT] Mission start warning (maybe no mission uploaded): {e}")

async def main():
    drone = System()
    print(f"[DRONE] Connecting to {MAVSDK_CONNECTION} ...")
    await drone.connect(system_address=MAVSDK_CONNECTION)


    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[DRONE] Connected!")
            break

    print("[DRONE] Ready. Starting automatic arm/takeoff to mission cruise altitude.")
    await arm_and_takeoff_to_cruise(drone)

    try:
        await mission_command_loop(drone)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f"[MAIN] Exception in command loop: {e}")

    try:
        await drone.offboard.stop()
    except Exception:
        pass

    print("[MAIN] Script finished. Disconnecting.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user.")
    except Exception as e:
        print(f"\n[MAIN] Error: {e}")
