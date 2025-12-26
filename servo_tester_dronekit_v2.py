#!/usr/bin/env python3
import asyncio
import collections
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import aioconsole

# Connection
CONNECTION_STRING = "/dev/ttyUSB0"  # Or /dev/ttyACM0
BAUD_RATE = 57600  # Try 57600 if needed

# Servo config
SERVO_CHANNELS = [9, 10, 11, 12, 13]  # AUX1 to AUX5
SERVO_HIGH_PWM = 1900
SERVO_NEUTRAL_PWM = 1500
ROTATE_TIME = 1.0
DELAY_BETWEEN = 2.0

def set_servo(vehicle, channel, pwm):
    """Use pymavlink to send MAV_CMD_DO_SET_SERVO via DroneKit's message_factory"""
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,       # confirmation
        channel, # param1: servo channel
        pwm,     # param2: PWM value
        0, 0, 0, 0, 0  # unused params
    )
    vehicle.send_mavlink(msg)

async def rotate_all_servos(vehicle):
    print("\nStarting sequential activation of 5 servos...\n")
    for i, channel in enumerate(SERVO_CHANNELS):
        print(f"Activating AUX{i+1} (channel {channel})")
        set_servo(vehicle, channel, SERVO_HIGH_PWM)
        await asyncio.sleep(ROTATE_TIME)
        set_servo(vehicle, channel, SERVO_NEUTRAL_PWM)
        print(f"AUX{i+1} returned to neutral")
        if i < 4:
            await asyncio.sleep(DELAY_BETWEEN - ROTATE_TIME)
    print("\nAll 5 servos completed!\n")

async def main():
    print(f"Connecting to {CONNECTION_STRING} @ {BAUD_RATE} baud...")
    try:
        vehicle = connect(CONNECTION_STRING, baud=BAUD_RATE, wait_ready=True, heartbeat_timeout=30)
    except Exception as e:
        print(f"Connection failed: {e}")
        print("Tips:")
        print("  - Check port: ls /dev/tty*")
        print("  - Try baud=57600")
        print("  - Close QGC/Mission Planner")
        print("  - Add user to dialout group")
        return

    print("Connected successfully!")

    # Optional: Print some vehicle info
    print(f"Vehicle version: {vehicle.version}")
    print(f"Autopilot Firmware version: {vehicle.version}")

    print("\nReady!")
    print("Type 'h' + Enter → Activate all 5 servos sequentially")
    print("Type 'q' + Enter → Quit\n")

    try:
        while True:
            cmd = (await aioconsole.ainput("> ")).strip().lower()
            if cmd == "h":
                await rotate_all_servos(vehicle)
            elif cmd == "q":
                print("Goodbye!")
                break
            else:
                print("Unknown command — use 'h' or 'q'")
    finally:
        vehicle.close()
        print("Connection closed.")

if __name__ == "__main__":
    asyncio.run(main())
