#!/usr/bin/env python3
import asyncio

import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode
import aioconsole
from pymavlink import mavutil


CONNECTION_STRING = "/dev/ttyUSB0"  # Change to /dev/ttyACM0 if needed

SERVO_CHANNELS = [9, 10, 11, 12, 13]
SERVO_NEUTRAL_PWM = 1500
SERVO_HIGH_PWM = 1900
ROTATE_DURATION = 1.0
DELAY_BETWEEN_SERVOS = 2.0

async def rotate_servos_sequentially(vehicle):
    print("\nStarting sequential servo activation (5 servos)...\n")
    for i, channel in enumerate(SERVO_CHANNELS):
        print(f"Activating AUX{i+1} (channel {channel})")
        print("Number of channels:", len(vehicle.channels))
        print("Channels dict:", dict(vehicle.channels))
        print("Overrides dict:", vehicle.channels.overrides)
        vehicle.channels.overrides[str(channel)] = SERVO_HIGH_PWM
        #vehicle.channels.overrides[str(channel)] = SERVO_HIGH_PWM
        await asyncio.sleep(ROTATE_DURATION)
        vehicle.channels.overrides[str(channel)] = SERVO_NEUTRAL_PWM
        if i < 4:
            await asyncio.sleep(DELAY_BETWEEN_SERVOS - ROTATE_DURATION)
    vehicle.channels.overrides = {}
    print("\nAll 5 servos completed and returned to neutral.\n")

async def main():
    print(f"Connecting to vehicle on: {CONNECTION_STRING}")
    try:
        vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=57600, heartbeat_timeout=30)
    except Exception as e:
        print(f"Connection failed: {e}")
        print("Tips:")
        print("  - Check port: ls /dev/tty*")
        print("  - Try baud=115200")
        print("  - Close QGC/Mission Planner")
        print("  - Run with sudo or add user to dialout group")
        return

    print("Connected successfully!")

    print("\nReady!")
    print("   Type 'h' + Enter → Rotate all 5 servos sequentially")
    print("   Type 'q' + Enter → Quit\n")

    while True:
        cmd = (await aioconsole.ainput("> ")).strip().lower()
        if cmd == "h":
            await rotate_servos_sequentially(vehicle)
        elif cmd == "q":
            print("Goodbye!")
            break
        else:
            print("Unknown command — use 'h' or 'q'")

    vehicle.close()

if __name__ == "__main__":
    asyncio.run(main())
