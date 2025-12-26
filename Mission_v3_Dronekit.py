#!/usr/bin/env python3
import asyncio
import time
import collections
import collections.abc

collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
import aioconsole
from pymavlink import mavutil

CONNECTION_STRING = "/dev/ttyUSB0"
BAUD_RATE = 57600

CRUISE_ALT = 10.0
HOLD_DESCEND_ALT = 5.0
HOLD_TIME = 10
HOLD_DESCENT_SPEED = 0.7
HOLD_ASCENT_SPEED = 0.8
MAX_HOLDS_BEFORE_RTL = 5

SERVO_CHANNELS = [1, 10, 11, 12, 13]
SERVO_NEUTRAL_PWM = 1500
SERVO_HIGH_PWM = 1900
ROTATE_DURATION = 1.0
DELAY_BETWEEN_SERVOS = 2.0

async def get_current_altitude(vehicle):
    while (vehicle.location.global_relative_frame is None or 
           vehicle.location.global_relative_frame.alt is None):
        await asyncio.sleep(0.1)
    return vehicle.location.global_relative_frame.alt

async def set_velocity_body(vehicle, vx=0.0, vy=0.0, vz=0.0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)

async def go_to_altitude(vehicle, target_alt, speed, tolerance=0.3, timeout=30.0):
    print(f"[ALT] Going to altitude: {target_alt:.2f} m")
    start_time = time.time()
    while True:
        if time.time() - start_time > timeout:
            print("[ERROR] Altitude timeout!")
            break
        current_alt = await get_current_altitude(vehicle)
        error = target_alt - current_alt
        if abs(error) <= tolerance:
            print(f"[ALT] Reached target altitude ({current_alt:.2f} m)")
            break
        vz = -speed if error > 0 else speed
        await set_velocity_body(vehicle, vz=vz)
        await asyncio.sleep(0.2)
    await set_velocity_body(vehicle, vz=0.0)

async def rotate_servos_sequentially(vehicle):
    for i, channel in enumerate(SERVO_CHANNELS):
        print(f"[SERVO] Activating AUX{i+1} (channel {channel})")
        vehicle.channels.overrides[str(channel)] = SERVO_HIGH_PWM
        await asyncio.sleep(ROTATE_DURATION)
        vehicle.channels.overrides[str(channel)] = SERVO_NEUTRAL_PWM
        if i < len(SERVO_CHANNELS) - 1:
            await asyncio.sleep(DELAY_BETWEEN_SERVOS - ROTATE_DURATION)

async def descend_hold_ascend(vehicle):
    print("[MODE] Switching to GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        await asyncio.sleep(0.1)

    try:
        await go_to_altitude(vehicle, HOLD_DESCEND_ALT, HOLD_DESCENT_SPEED)

        print("[HOLD] Starting 10-second hold with servo activation")
        await rotate_servos_sequentially(vehicle)

        remaining = HOLD_TIME - (len(SERVO_CHANNELS) * DELAY_BETWEEN_SERVOS)
        if remaining > 0:
            print(f"[HOLD] Hovering for extra {remaining:.1f}s")
            await asyncio.sleep(remaining)

        await go_to_altitude(vehicle, CRUISE_ALT, HOLD_ASCENT_SPEED)

    finally:
        vehicle.channels.overrides = {}
        print("[MODE] Returning to AUTO to resume mission")
        vehicle.mode = VehicleMode("AUTO")
        while vehicle.mode.name != "AUTO":
            await asyncio.sleep(0.1)

    print("[HOLD] Hold complete – mission resumed")
    print("=" * 60)
    return True

async def mission_command_loop(vehicle):
    print("\n[INFO] Ready! Commands:")
    print("   'h' → Trigger hold + servo sequence")
    print("   'q' → Command RTL and quit\n")
    holds = 0

    while True:
        cmd = (await aioconsole.ainput("> ")).strip().lower()
        if cmd == "h":
            holds += 1
            print(f"\n[INFO] Hold #{holds}/{MAX_HOLDS_BEFORE_RTL}")
            await descend_hold_ascend(vehicle)
            if holds >= MAX_HOLDS_BEFORE_RTL:
                print("[FINAL] Max holds reached – going RTL")
                vehicle.mode = VehicleMode("RTL")
                break
        elif cmd == "q":
            print("[INFO] Commanding RTL")
            vehicle.mode = VehicleMode("RTL")
            break
        else:
            print("[WARN] Unknown command")

async def main():
    print(f"[DRONE] Connecting to {CONNECTION_STRING} @ {BAUD_RATE} baud...")
    try:
        vehicle = connect(CONNECTION_STRING, wait_ready=True, heartbeat_timeout=30)
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        print("Tips:")
        print("  - Check cable and port: ls /dev/tty*")
        print("  - Try baud=115200 or 57600")
        print("  - Run with sudo or add user to dialout group")
        return

    print("[DRONE] Connected successfully!")

    print("[INFO] Arm and fly your mission in Mission Planner (switch to AUTO)")
    print("[INFO] Then press 'h' here to trigger holds with servo activation")

    try:
        await mission_command_loop(vehicle)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        vehicle.close()
        print("[MAIN] Connection closed – goodbye!")

if __name__ == "__main__":
    asyncio.run(main())
