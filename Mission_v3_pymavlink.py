#!/usr/bin/env python3
import asyncio
import time
from pymavlink import mavutil
import aioconsole

# Connection
CONNECTION = '/dev/ttyUSB0'  # Change to /dev/ttyACM0 if needed
BAUD = 57600                # Common for Pixhawk USB; try 57600 if needed

# Mission parameters
CRUISE_ALT = 10.0
HOLD_DESCEND_ALT = 5.0
HOLD_TIME = 10
HOLD_DESCENT_SPEED = 0.7
HOLD_ASCENT_SPEED = 0.8
MAX_HOLDS_BEFORE_RTL = 5

# Servo settings
SERVO_CHANNELS = [9, 10, 11, 12, 13]  # AUX1 to AUX5 (channels 9-13)
SERVO_HIGH_PWM = 1900
SERVO_NEUTRAL_PWM = 1050
ROTATE_DURATION = 2.0  # Time to hold servo rotated (increased to make it visible)
HOLD_SERVO_DURATION = 8.0  # Remaining hold time after servo rotation

master = None

def set_velocity_body(vx=0.0, vy=0.0, vz=0.0):
    global master
    if master is not None:
        msg = master.mav.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0, 0, 0
        )
        master.mav.send(msg)

def set_servo_pwm(channel: int, pwm: int):
    global master
    if master is not None:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            channel,
            pwm,
            0, 0, 0, 0, 0
        )

async def stream_zero_velocity(duration_s: float):
    interval = 0.05
    count = int(duration_s / interval)
    for _ in range(count):
        set_velocity_body(0.0, 0.0, 0.0)
        await asyncio.sleep(interval)

async def get_current_altitude():
    global master
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if msg:
            return msg.relative_alt / 1000.0
        await asyncio.sleep(0.1)

async def go_to_altitude(target_alt: float, speed: float, tolerance: float = 0.3, timeout: float = 20.0):
    print(f"[ALT] Target altitude: {target_alt:.2f} m")
    start = time.time()
    while True:
        if time.time() - start > timeout:
            print("[ERROR] Altitude change timeout")
            break
        alt = await get_current_altitude()
        error = target_alt - alt
        if abs(error) <= tolerance:
            print(f"[ALT] Reached target altitude ({alt:.2f} m)")
            break
        vz = -speed if error > 0 else speed
        vz = max(min(vz, 1.5), -1.5)
        set_velocity_body(vz=vz)
        await asyncio.sleep(0.25)
    await stream_zero_velocity(0.3)

async def activate_one_servo(hold_number: int):
    """Activate only the corresponding servo for this hold (1st hold → AUX1, 2nd → AUX2, etc.) mm  so h is used here"""
    servo_index = hold_number - 1  # 0-based index
    channel = SERVO_CHANNELS[servo_index]
    print(f"[SERVO] Hold #{hold_number}: Activating only AUX{hold_number} (channel {channel})")
    set_servo_pwm(channel, SERVO_HIGH_PWM)
    await asyncio.sleep(ROTATE_DURATION)
    #set_servo_pwm(channel, SERVO_NEUTRAL_PWM)
    print(f"[SERVO] AUX{hold_number} returned to neutral")
    # Hover for the remaining hold time
    await stream_zero_velocity(HOLD_SERVO_DURATION)

async def descend_hold_ascend(hold_number: int):
    print("[MODE] Switching to GUIDED")
    global master
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # Guided mode in ArduCopter
    )
    await asyncio.sleep(1.0)

    try:
        await go_to_altitude(HOLD_DESCEND_ALT, HOLD_DESCENT_SPEED)
        await activate_one_servo(hold_number)  # Only one servo per hold
        await go_to_altitude(CRUISE_ALT, HOLD_ASCENT_SPEED)
        print("[TRANSITION] Streaming zero velocity for handover...")
        await stream_zero_velocity(5.0)
    finally:
        print("[MODE] Returning to AUTO")
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            3  # Auto mode in ArduCopter
        )
        await asyncio.sleep(1.0)

    print("[HOLD] Hold sequence complete – mission resumed")
    print("=" * 60)
    return True

async def mission_command_loop():
    print("[INFO] Ready for commands")
    print("[INFO]  click only these  --  h = hold | q = land")
    holds = 0
    while True:
        cmd = (await aioconsole.ainput("> ")).strip().lower()
        if cmd == "h":
            set_servo_pwm(14,SERVO_HIGH_PWM)
            await asyncio.sleep(1)
            set_servo_pwm(14,1050)
            holds += 1
            print(f"[INFO] Hold #{holds}/{MAX_HOLDS_BEFORE_RTL}")
            success = await descend_hold_ascend(holds)
            if holds >= MAX_HOLDS_BEFORE_RTL and success:
                print("[FINAL] 5th hold completed")
                print("[FINAL] Waiting 10 seconds at mission altitude")
                for i in range(10):
                    print(f"[FINAL] Wait {i + 1}/10")
                    await asyncio.sleep(1)
                print("[FINAL] Commanding RTL")
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                break
        elif cmd == "q":
            print("[INFO] Landing requested")
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            break
        else:
            print("[WARN] Unknown command")

async def main():
    global master
    print(f"[DRONE] Connecting to {CONNECTION} @ {BAUD} baud...")
    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD)

    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Connected successfully!")

    print("[INFO] Mission NOT auto-started")
    print("[INFO] Start mission manually from QGC/Mission Planner by switching to AUTO mode")
    print("[INFO] During flight, press 'h' to trigger hold — only one servo per hold (1st=AUX1, 2nd=AUX2, etc.)")

    await mission_command_loop()

    print("[MAIN] Script finished")
    print("[MAIN] junu u are great .........")

if __name__ == "__main__":
    asyncio.run(main())
