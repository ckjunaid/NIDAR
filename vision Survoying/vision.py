import cv2
import numpy as np
from pyzbar import pyzbar  # pip install pyzbar
import time
from pymavlink import mavutil

# ============================
# Configuration
# ============================
CONNECTION = 'udpin:0.0.0.0:14540'   # PX4 offboard port (QGC uses 14550)
TARGET_ALT = 5.0                    # Target inspection altitude
DESCEND_SPEED = 0.8                 # m/s downward
COOLDOWN_AFTER_INSPECTION = 15       # seconds before same person can trigger again

PID_KP = 0.6
PID_KD = 0.15
DEADZONE = 30
VEL_SCALE = 120.0
YAW_SCALE = 250.0
CAM_INDEX = 0

# ============================
# Connect to PX4
# ============================
print("Connecting to PX4 on offboard port 14540...")
master = mavutil.mavlink_connection(CONNECTION)
master.wait_heartbeat()
print("Connected!")

# ============================
# Robust Mode Mapping
# ============================
mode_map = master.mode_mapping()
if mode_map is None:
    print("ERROR: No mode mapping available – check connection")
    exit()

print("Available modes:", list(mode_map.keys()))

AUTO_MODE = mode_map['MISSION']
OFFBOARD_MODE = mode_map['OFFBOARD']

print(f"AUTO = {AUTO_MODE}, OFFBOARD = {OFFBOARD_MODE}")

# ============================
# MAVLink Helpers
# ============================
def send_body_velocity(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111110000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate
    )

def send_hover():
    send_body_velocity(0, 0, 0, 0)

def set_mode(mode_id):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

# ============================
# PID Controller (time-based)
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
        if np.isnan(output) or np.isinf(output):
            return 0.0
        return output

pid_x = PID(PID_KP, PID_KD)
pid_y = PID(PID_KP, PID_KD)

# ============================
# Camera Setup
# ============================
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ============================
# State Machine
# ============================
state = "MONITORING"
last_inspection_time = 0
last_time = time.time()
current_alt = 10.0  # initial estimate

print("\n=== PX4 Person Inspection Survey ===")
print("Start your survey mission in QGC (AUTO mode at 10m+)")
print("Script is monitoring for person (QR code)")

# ============================
# Main Loop
# ============================
try:
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        cx = w // 2
        cy = h // 2

        # Update altitude from PX4
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            current_alt = -msg.z  # NED: negative Z = altitude

        qrs = pyzbar.decode(frame)

        vf = 0.0
        vr = 0.0
        vz = 0.0
        yr = 0.0

        # ============================
        # MONITORING (mission in AUTO)
        # ============================
        if state == "MONITORING":
            cv2.putText(frame, "MONITORING (Mission Active)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cooldown_left = max(0, int(COOLDOWN_AFTER_INSPECTION - (now - last_inspection_time)))
            cv2.putText(frame, f"Cooldown: {cooldown_left}s", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            if qrs and (now - last_inspection_time > COOLDOWN_AFTER_INSPECTION):
                print("\n=== PERSON DETECTED ===")
                print("Pre-streaming setpoints and switching to OFFBOARD...")

                # Long pre-stream (8 seconds)
                for _ in range(160):
                    send_hover()
                    time.sleep(0.05)

                set_mode(OFFBOARD_MODE)

                # Confirm OFFBOARD
                offboard_ok = False
                for _ in range(100):
                    hb = master.recv_match(type='HEARTBEAT', blocking=False)
                    if hb and hb.custom_mode == OFFBOARD_MODE:
                        offboard_ok = True
                        break
                    send_hover()
                    time.sleep(0.05)

                if offboard_ok:
                    print("OFFBOARD active – descending to 5m while centering")
                    state = "DESCENDING"
                else:
                    print("OFFBOARD switch failed – check arming/RC/parameters")
                    continue

        # ============================
        # DESCENDING (OFFBOARD active)
        # ============================
        elif state == "DESCENDING":
            cv2.putText(frame, "DESCENDING + CENTERING", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, f"Altitude: {current_alt:.1f}m → 5.0m", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Safety: if OFFBOARD lost, return to AUTO
            hb = master.recv_match(type='HEARTBEAT', blocking=False)
            if hb and hb.custom_mode != OFFBOARD_MODE:
                print("OFFBOARD lost – returning to AUTO")
                set_mode(AUTO_MODE)
                state = "MONITORING"
                continue

            vz = DESCEND_SPEED

            if qrs:
                qr = qrs[0]
                pts = np.array([(p.x, p.y) for p in qr.polygon], dtype=np.float32)
                qx, qy = np.mean(pts, axis=0)

                err_x = qx - cx
                err_y = qy - cy

                cv2.polylines(frame, [pts.astype(int)], True, (0, 255, 0), 3)
                cv2.circle(frame, (int(qx), int(qy)), 8, (0, 0, 255), -1)
                cv2.putText(frame, "CENTERING PERSON", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                if abs(err_x) > DEADZONE:
                    vr = -pid_x.compute(err_x, now) / VEL_SCALE
                    yr = -pid_x.compute(err_x, now) / YAW_SCALE
                if abs(err_y) > DEADZONE:
                    vf = pid_y.compute(err_y, now) / VEL_SCALE

            send_body_velocity(vf, vr, vz, yr)

            # Reached target altitude
            if current_alt <= TARGET_ALT + 0.3:
                print("Reached 5m – Resuming mission (AUTO mode)")
                set_mode(AUTO_MODE)
                last_inspection_time = now
                state = "MONITORING"

        # Visual center aid
        cv2.circle(frame, (cx, cy), DEADZONE, (255, 255, 255), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

        cv2.imshow("PX4 Person Inspection Survey", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("\nStopping script safely...")
    send_hover()
    time.sleep(1)
    cap.release()
    cv2.destroyAllWindows()
    set_mode(AUTO_MODE)
    print("Script ended — mission can continue in QGC")
