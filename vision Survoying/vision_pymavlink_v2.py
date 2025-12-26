import cv2
import numpy as np
from pyzbar import pyzbar  # pip install pyzbar
import time
from pymavlink import mavutil

# ============================
# Configuration
# ============================
SERIAL_PORT = '/dev/ttyUSB0'        # Change if needed
BAUD_RATE = 57600
TARGET_ALTITUDE = 5.0
DESCEND_SPEED = 0.8                 # max downward speed (m/s)
HOLD_DURATION = 15                  # seconds at 5m
INSPECTION_COOLDOWN = 30            # seconds before re-trigger

PID_KP = 0.6
PID_KD = 0.15
DEADZONE = 30
VEL_SCALE = 120.0
YAW_SCALE = 250.0
CAM_INDEX = 0

# ============================
# Connect to ArduCopter
# ============================
print(f"Connecting to ArduCopter on {SERIAL_PORT} at {BAUD_RATE} baud...")
master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
master.wait_heartbeat()
print("Connected!")

# ============================
# Mode Mapping
# ============================
mode_map = master.mode_mapping()
if mode_map is None:
    print("No mode mapping – using defaults")
    AUTO_MODE = 3
    GUIDED_MODE = 4
else:
    AUTO_MODE = mode_map['AUTO']
    GUIDED_MODE = mode_map['GUIDED']

print(f"AUTO: {AUTO_MODE}, GUIDED: {GUIDED_MODE}")

# ============================
# Velocity command
# ============================


def send_body_velocity(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate
    )


def send_hover():
    send_body_velocity()


def set_mode(mode_id):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        int(mode_id)
    )

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
# Camera
# ============================
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ============================
# State Machine
# ============================
state = "MONITORING"
last_inspection_time = 0
inspection_start_time = 0
current_alt = 10.0  # initial (will be updated from drone)

print("Start mission in AUTO mode. Monitoring for person...")

try:
    while True:
        now = time.time()

        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        cx = w // 2
        cy = h // 2

        # Get real altitude from drone
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # mm → meters

        qrs = pyzbar.decode(frame)

        vf = 0.0
        vr = 0.0
        vz = 0.0
        yr = 0.0

        if state == "MONITORING":
            cv2.putText(frame, "MONITORING (AUTO)", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cooldown = max(0, int(INSPECTION_COOLDOWN -
                           (now - last_inspection_time)))
            cv2.putText(frame, f"Cooldown: {cooldown}s", (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            if qrs and (now - last_inspection_time > INSPECTION_COOLDOWN):
                print("Person detected – switching to GUIDED")
                set_mode(GUIDED_MODE)
                state = "DESCENDING"
                inspection_start_time = now

        elif state == "DESCENDING":
            cv2.putText(frame, "DESCENDING + CENTERING", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, f"Real Alt: {current_alt:.1f}m → 5.0m", (
                20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            vz = DESCEND_SPEED  # constant downward

            if qrs:
                qr = qrs[0]
                pts = np.array([(p.x, p.y)
                               for p in qr.polygon], dtype=np.float32)
                qx = np.mean(pts[:, 0])
                qy = np.mean(pts[:, 1])

                err_x = qx - cx
                err_y = qy - cy

                cv2.polylines(frame, [pts.astype(int)], True, (0, 255, 0), 3)
                cv2.circle(frame, (int(qx), int(qy)), 8, (0, 0, 255), -1)

                if abs(err_x) > DEADZONE:
                    vr = -pid_x.compute(err_x, now) / VEL_SCALE
                    yr = -pid_x.compute(err_x, now) / YAW_SCALE
                if abs(err_y) > DEADZONE:
                    vf = pid_y.compute(err_y, now) / VEL_SCALE

            send_body_velocity(vf, vr, vz, yr)

            # Check real altitude
            if current_alt <= TARGET_ALTITUDE + 0.3:
                print("Reached 5m (real alt) – Starting 15s hold")
                state = "HOLDING"
                inspection_start_time = now

        elif state == "HOLDING":
            remaining = int(HOLD_DURATION - (now - inspection_start_time))
            cv2.putText(frame, "HOLDING AT 5m", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"Hold: {max(0, remaining)}s | Alt: {current_alt:.1f}m", (
                20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            vz = 0.0  # hover vertically

            if qrs:
                qr = qrs[0]
                pts = np.array([(p.x, p.y)
                               for p in qr.polygon], dtype=np.float32)
                qx = np.mean(pts[:, 0])
                qy = np.mean(pts[:, 1])

                err_x = qx - cx
                err_y = qy - cy

                cv2.polylines(frame, [pts.astype(int)], True, (0, 255, 0), 3)
                cv2.circle(frame, (int(qx), int(qy)), 8, (0, 0, 255), -1)

                if abs(err_x) > DEADZONE:
                    vr = -pid_x.compute(err_x, now) / VEL_SCALE
                    yr = -pid_x.compute(err_x, now) / YAW_SCALE
                if abs(err_y) > DEADZONE:
                    vf = pid_y.compute(err_y, now) / VEL_SCALE

            send_body_velocity(vf, vr, vz, yr)

            if now - inspection_start_time > HOLD_DURATION:
                print("Hold complete – Returning to AUTO mission")
                set_mode(AUTO_MODE)
                last_inspection_time = now
                state = "MONITORING"

        # Visual aids
        cv2.circle(frame, (cx, cy), DEADZONE, (255, 255, 255), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

        cv2.imshow("ArduPilot Person Inspection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("\nStopping...")
    send_hover()
    time.sleep(1)
    set_mode(AUTO_MODE)
    cap.release()
    cv2.destroyAllWindows()
    print("Script ended — mission continues")
