#!/usr/bin/env python3
import asyncio
from pymavlink import mavutil
import aioconsole

# Connection - change if your port is /dev/ttyACM0
CONNECTION = '/dev/ttyUSB0'
BAUD = 57600  # Common for Pixhawk USB; try 57600 if no connection

# Servo config (AUX1 to AUX5 = channels 9 to 13)
SERVO_CHANNELS = [9, 10, 11, 12, 13, 14]
SERVO_HIGH_PWM = 1950   # Full rotation
SERVO_NEUTRAL_PWM = 1500
ROTATE_DURATION = 2  # Hold high position for 1 second
DELAY_BETWEEN = 2.0     # Time from start of one servo to next

def set_servo(master, channel, pwm):
    """Send MAV_CMD_DO_SET_SERVO to control a servo"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # confirmation
        channel,  # param1: servo channel
        pwm,      # param2: PWM value
        0, 0, 0, 0, 0  # unused params
    )

async def rotate_servos_sequentially(master):
    print("\nStarting sequential servo activation...\n")
    for i, channel in enumerate(SERVO_CHANNELS):
        print(f"Activating AUX{i+1} (channel {channel})")
        set_servo(master, channel, SERVO_HIGH_PWM)
        await asyncio.sleep(ROTATE_DURATION)
        #set_servo(master, channel, SERVO_NEUTRAL_PWM)
        if i < 4:
            await asyncio.sleep(DELAY_BETWEEN - ROTATE_DURATION)
    print("\nAll 5 servos completed and returned to neutral.\n")

async def main():
    print(f"Connecting to {CONNECTION} @ {BAUD} baud...")
    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)

    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received — connected to vehicle!")

    print("\nReady!")
    print("Type 'h' + Enter → Activate all 5 servos sequentially")
    print("Type 'q' + Enter → Quit\n")

    while True:
        cmd = (await aioconsole.ainput("> ")).strip().lower()
        if cmd == "h":
            await rotate_servos_sequentially(master)
        elif cmd == "q":
            print("Goodbye!")
            break
        else:
            print("Unknown command — use 'h' or 'q'")

if __name__ == "__main__":
    asyncio.run(main())
