#!/usr/bin/env python3
import asyncio
from mavsdk import System
import aioconsole

CONNECTION = "serial:///dev/ttyUSB0:57600"  # Adjust if needed

SERVO_CHANNELS = [9, 10, 11, 12, 13]  # AUX1 to AUX5
SERVO_HIGH_PWM = 1900
SERVO_NEUTRAL_PWM = 1500
ROTATE_TIME = 1.0
DELAY_BETWEEN = 2.0

async def set_servo_pwm(drone: System, channel: int, pwm: int):
    """Send raw MAV_CMD_DO_SET_SERVO using low-level command (works on ArduPilot)"""
    await drone._async_send_command_long(
        target_system=1,  # Usually 1 for autopilot
        target_component=1,
        command=183,  # MAV_CMD_DO_SET_SERVO = 183
        confirmation=0,
        param1=channel,
        param2=pwm,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0
    )

async def rotate_all_servos(drone: System):
    print("\nStarting sequential activation of 5 servos...\n")
    for i, channel in enumerate(SERVO_CHANNELS):
        print(f"Activating AUX{i+1} (channel {channel})")
        await set_servo_pwm(drone, channel, SERVO_HIGH_PWM)
        await asyncio.sleep(ROTATE_TIME)
        await set_servo_pwm(drone, channel, SERVO_NEUTRAL_PWM)
        print(f"AUX{i+1} returned to neutral")
        if i < 4:
            await asyncio.sleep(DELAY_BETWEEN - ROTATE_TIME)
    print("\nAll 5 servos completed!\n")

async def main():
    drone = System()
    print(f"Connecting to drone on {CONNECTION}...")
    await drone.connect(system_address=CONNECTION)

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected successfully!")
            break

    print("\nReady!")
    print("Type 'h' + Enter → Activate all 5 servos sequentially")
    print("Type 'q' + Enter → Quit\n")

    while True:
        cmd = (await aioconsole.ainput("> ")).strip().lower()
        if cmd == "h":
            await rotate_all_servos(drone)
        elif cmd == "q":
            print("Goodbye!")
            break
        else:
            print("Unknown command — use 'h' or 'q'")

if __name__ == "__main__":
    asyncio.run(main())
