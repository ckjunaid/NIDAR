#!/usr/bin/env python3
import asyncio
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

CONNECTION_STRING = "udp://:14540" 

CRUISE_ALT = 10.0
HOLD_DESCEND_ALT = 5.0
HOLD_TIME = 10
HOLD_DESCENT_SPEED = 0.7
HOLD_ASCENT_SPEED = 0.8
MAX_HOLDS_BEFORE_RTL = 5

ACTUATOR_INDICES = [1, 2, 3, 4, 5]  
ACTUATOR_HIGH = 1.0
ACTUATOR_NEUTRAL = 0.0
ROTATE_DURATION = 1.0
DELAY_BETWEEN_ACTUATORS = 2.0

async def stream_zero_velocity(drone, duration):
    end_time = time.time() + duration
    while time.time() < end_time:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.05)

async def go_to_altitude(drone, target_alt, speed, tolerance=0.3):
    print(f"[ALT] Targeting {target_alt:.2f} m")
    async for position in drone.telemetry.position():
        current_alt = position.relative_altitude_m
        error = target_alt - current_alt
        if abs(error) <= tolerance:
            print(f"[ALT] Reached {current_alt:.2f} m")
            break
        vz = -speed if error > 0 else speed
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, vz, 0.0))
        await asyncio.sleep(0.2)
    await stream_zero_velocity(drone, 0.5)

async def activate_actuators_sequentially(drone):
    for i, index in enumerate(ACTUATOR_INDICES):
        print(f"[ACTUATOR] Activating Offboard Actuator {index} (AUX{i+1})")
        await drone.action.set_actuator(index, ACTUATOR_HIGH)
        await asyncio.sleep(ROTATE_DURATION)
        await drone.action.set_actuator(index, ACTUATOR_NEUTRAL)
        if i < len(ACTUATOR_INDICES) - 1:
            await asyncio.sleep(DELAY_BETWEEN_ACTUATORS - ROTATE_DURATION)

async def descend_hold_ascend(drone):
    print("[MODE] Starting Offboard")
    await stream_zero_velocity(drone, 1.0)
    await drone.offboard.start()

    try:
        await go_to_altitude(drone, HOLD_DESCEND_ALT, HOLD_DESCENT_SPEED)
        print("[HOLD] Starting 10-second hold with actuator activation")
        await activate_actuators_sequentially(drone)
        remaining = HOLD_TIME - (len(ACTUATOR_INDICES) * DELAY_BETWEEN_ACTUATORS)
        if remaining > 0:
            await stream_zero_velocity(drone, remaining)
        await go_to_altitude(drone, CRUISE_ALT, HOLD_ASCENT_SPEED)
        await stream_zero_velocity(drone, 3.0)
    finally:
        await drone.offboard.stop()
        await drone.action.set_flight_mode("AUTO")
        print("[MODE] Returned to AUTO")

    print("[HOLD] Hold complete – mission resumed")
    print("=" * 60)
    return True

async def mission_command_loop(drone):
    print("\n[INFO] Ready! Commands:")
    print(" 'h' → Trigger hold + actuator sequence")
    print(" 'q' → RTL and quit\n")
    holds = 0
    while True:
        cmd = input("> ").strip().lower()
        if cmd == "h":
            holds += 1
            print(f"\n[INFO] Hold #{holds}/{MAX_HOLDS_BEFORE_RTL}")
            await descend_hold_ascend(drone)
            if holds >= MAX_HOLDS_BEFORE_RTL:
                print("[FINAL] Max holds reached – RTL")
                await drone.action.return_to_launch()
                break
        elif cmd == "q":
            await drone.action.return_to_launch()
            break

async def main():
    drone = System()
    print(f"[DRONE] Connecting to PX4 SITL on {CONNECTION_STRING}...")
    await drone.connect(system_address=CONNECTION_STRING)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[DRONE] Connected to PX4 SITL!")
            break

    print("[INFO] In QGC: Arm, takeoff, upload mission, switch to AUTO")
    print("[INFO] Then press 'h' here to interrupt with hold + actuators")

    await mission_command_loop(drone)

if __name__ == "__main__":
    asyncio.run(main())
