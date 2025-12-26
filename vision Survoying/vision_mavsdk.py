import asyncio
import cv2
import numpy as np
from pyzbar import pyzbar
import time
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

# ============================
# Configuration
# ============================
CONNECTION = "udp://:14540"
TARGET_ALTITUDE = 5.0
DESCEND_SPEED = 0.8
HOLD_DURATION = 15
INSPECTION_COOLDOWN = 30

PID_KP = 0.6
PID_KD = 0.15
DEADZONE = 30
VEL_SCALE = 120.0
YAW_SCALE = 250.0

# Global targets for streaming task
target_forward = 0.0
target_right = 0.0
target_down = 0.0
target_yaw_rate = 0.0

# ============================
# Continuous Setpoint Streaming (50 Hz)
# ============================


async def setpoint_stream(drone):
    while True:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(target_forward, target_right,
                                 target_down, target_yaw_rate)
        )
        await asyncio.sleep(0.02)

# ============================
# PID
# ============================


class PID:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0.0
        self.last_time = None

    def compute(self, error, current_time):
        if self.last_time is None:
            dt = 0.033
        else:
            dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0:
            dt = 0.033

        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.kd * derivative
        return 0.0 if np.isnan(output) or np.isinf(output) else output


pid_x = PID(PID_KP, PID_KD)
pid_y = PID(PID_KP, PID_KD)

# ============================
# Main
# ============================


async def main():
    global target_forward, target_right, target_down, target_yaw_rate

    drone = System()
    await drone.connect(system_address=CONNECTION)

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Waiting for position...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Position OK")
            break

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    state = "MONITORING"
    last_inspection_time = 0
    inspection_start_time = 0

    print("Start mission in QGC (AUTO). Monitoring for person...")

    streaming_task = asyncio.create_task(setpoint_stream(drone))

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            h, w = frame.shape[:2]
            cx = w // 2
            cy = h // 2

            now = time.time()

            position = await drone.telemetry.position().__anext__()
            current_alt = position.relative_altitude_m

            qrs = pyzbar.decode(frame)

            if state == "MONITORING":
                cv2.putText(frame, "MONITORING (AUTO)", (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                if qrs and (now - last_inspection_time > INSPECTION_COOLDOWN):
                    print("Person detected – starting Offboard")
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    await drone.offboard.start()
                    state = "DESCENDING"
                    inspection_start_time = now
                    last_inspection_time = now

            elif state == "DESCENDING":
                cv2.putText(frame, "DESCENDING + CENTERING", (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                target_down = DESCEND_SPEED

                target_forward = target_right = target_yaw_rate = 0.0

                if qrs:
                    qr = qrs[0]
                    pts = np.array([(p.x, p.y)
                                   for p in qr.polygon], dtype=np.float32)
                    qx = np.mean(pts[:, 0])
                    qy = np.mean(pts[:, 1])

                    err_x = qx - cx
                    err_y = qy - cy

                    cv2.polylines(frame, [pts.astype(int)],
                                  True, (0, 255, 0), 3)
                    cv2.circle(frame, (int(qx), int(qy)), 8, (0, 0, 255), -1)

                    if abs(err_x) > DEADZONE:
                        target_right = -pid_x.compute(err_x, now) / VEL_SCALE
                        target_yaw_rate = - \
                            pid_x.compute(err_x, now) / YAW_SCALE
                    if abs(err_y) > DEADZONE:
                        target_forward = pid_y.compute(err_y, now) / VEL_SCALE

                if current_alt <= TARGET_ALTITUDE + 0.3:
                    print("Reached 5m – Starting hold")
                    state = "HOLDING"
                    inspection_start_time = now

            elif state == "HOLDING":
                remaining = int(HOLD_DURATION - (now - inspection_start_time))
                cv2.putText(frame, "HOLDING AT 5m", (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(frame, f"Hold: {max(0, remaining)}s", (20, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                target_down = 0.0

                target_forward = target_right = target_yaw_rate = 0.0

                if qrs:
                    qr = qrs[0]
                    pts = np.array([(p.x, p.y)
                                   for p in qr.polygon], dtype=np.float32)
                    qx = np.mean(pts[:, 0])
                    qy = np.mean(pts[:, 1])

                    err_x = qx - cx
                    err_y = qy - cy

                    cv2.polylines(frame, [pts.astype(int)],
                                  True, (0, 255, 0), 3)
                    cv2.circle(frame, (int(qx), int(qy)), 8, (0, 0, 255), -1)

                    if abs(err_x) > DEADZONE:
                        target_right = -pid_x.compute(err_x, now) / VEL_SCALE
                        target_yaw_rate = - \
                            pid_x.compute(err_x, now) / YAW_SCALE
                    if abs(err_y) > DEADZONE:
                        target_forward = pid_y.compute(err_y, now) / VEL_SCALE

                # End hold and return to mission
                if now - inspection_start_time > HOLD_DURATION:
                    print("Hold complete – Stopping Offboard and returning to mission")
                    await drone.offboard.stop()  # ← This is the key line
                    state = "MONITORING"

            # Visual aids
            cv2.circle(frame, (cx, cy), DEADZONE, (255, 255, 255), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

            cv2.imshow("PX4 Person Inspection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        streaming_task.cancel()
        try:
            await drone.offboard.stop()
        except:
            pass
        cap.release()
        cv2.destroyAllWindows()
        print("Script ended — mission continues in QGC")

if __name__ == "__main__":
    asyncio.run(main())
