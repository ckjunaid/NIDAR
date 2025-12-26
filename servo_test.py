import asyncio
from mavsdk import System

# --- CONFIGURATION ---
# Change this to match your serial port and baud rate
SERIAL_PORT = "serial:///dev/ttyUSB0:57600" 
ROTATE_DURATION = 2.0  # Seconds to wait between servos

async def rotate_all_servos(drone):
    print("[SERVO] Starting sequential activation of 5 servos...")
    
    # ArduPilot Servo Channels (AUX 1-5 usually map to 9-13)
    servos = [9, 10, 11, 12, 13]
    
    for channel in servos:
        try:
            print(f"[SERVO] Channel {channel} -> 1900µs")
            
            # MAVSDK uses -1.0 to 1.0. 
            # For 1900µs, use 0.8 (1500 + 0.8 * 500)
            await drone.action.set_actuator(channel, 0.8)
            
            await asyncio.sleep(ROTATE_DURATION)
            
            # Return to neutral (1500µs)
            await drone.action.set_actuator(channel, -1.0)
            
        except Exception as e:
            print(f"[ERROR] Failed to set channel {channel}: {e}")

async def main():
    drone = System()
    
    print(f"Connecting to {SERIAL_PORT}...")
    await drone.connect(system_address=SERIAL_PORT)

    # Wait for connection
    print("Waiting for heartbeat...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to Flight Controller!")
            break

    await rotate_all_servos(drone)
    print("Sequence complete.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Test stopped by user.")
