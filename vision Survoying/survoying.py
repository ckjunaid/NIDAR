import cv2
import numpy as np
from pyzbar import pyzbar  # pip install pyzbar
import time
from pymavlink import mavutil

# ============================
# Connect to PX4 on offboard port 14540
# ============================
print("Connecting to PX4 on port 14540...")
master = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
master.wait_heartbeat()
print("Heartbeat received! Script in MONITORING mode (mission runs in AUTO)")

# ============================
# Send velocity in BODY frame
# ============================
def send_body_velocity(vx, vy, vz=0, yaw_rate=0):
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

# ============================
# Camera Setup
# ============================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ============================
# PID Controller
# ============================
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt=1/30.0):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

pid_x = PID(0.6, 0.0, 0.15)
pid_y = PID(0.6, 0.0, 0.15)

deadzone = 30
VEL_SCALE = 120.0
YAW_SCALE = 250.0

DESCEND_SPEED = 0.8
INSPECTION_DURATION = 10
QR_COOLDOWN = 20

# State machine
state = "MONITORING"          # "MONITORING" (AUTO) or "INSPECTING" (OFFBOARD)
last_qr_time = 0
inspection_start_time = 0

print("Start your survey mission in QGC now (AUTO mode).")
print("Script is monitoring camera – will take over only when QR is detected.")

# ============================
# Main Loop
# ============================
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        cx = w // 2
        cy = h // 2

        current_time = time.time()
        qrs = pyzbar.decode(frame)

        # Default display
        cv2.putText(frame, "STATE: MONITORING (Mission in AUTO)", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if state == "MONITORING":
            cooldown_remaining = max(0, int(QR_COOLDOWN - (current_time - last_qr_time)))
            cv2.putText(frame, f"QR Cooldown: {cooldown_remaining}s", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            if qrs and (current_time - last_qr_time > QR_COOLDOWN):
                print("QR DETECTED – Attempting switch to OFFBOARD")

                # Pre-stream hover
                for _ in range(50):
                    send_hover()
                    time.sleep(0.1)

                # Request OFFBOARD
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    6
                )

                # Wait up to 8 seconds for confirmation
                offboard_ok = False
                for _ in range(80):
                    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
                    if msg and msg.custom_mode == 6:
                        offboard_ok = True
                        break
                    send_hover()

                if offboard_ok:
                    print("Successfully in OFFBOARD – Starting inspection")
                    state = "INSPECTING"
                    inspection_start_time = current_time
                    last_qr_time = current_time
                else:
                    print("FAILED to switch to OFFBOARD – continuing monitoring")
                    # Continue in MONITORING

        elif state == "INSPECTING":
            cv2.putText(frame, "STATE: INSPECTING (OFFBOARD)", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            elapsed = int(current_time - inspection_start_time)
            cv2.putText(frame, f"Time: {elapsed}/{INSPECTION_DURATION}s", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            vf = 0.0
            vr = 0.0
            vz = DESCEND_SPEED
            yr = 0.0

            if qrs:
                qr = qrs[0]
                pts = np.array([(p.x, p.y) for p in qr.polygon], dtype=np.float32)
                qx = np.mean(pts[:, 0])
                qy = np.mean(pts[:, 1])

                err_x = qx - cx
                err_y = qy - cy

                cv2.polylines(frame, [pts.astype(int)], True, (0, 255, 0), 3)
                cv2.circle(frame, (int(qx), int(qy)), 8, (0, 0, 255), -1)
                cv2.putText(frame, "CENTERING QR", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                if abs(err_x) > deadzone or abs(err_y) > deadzone:
                    vf = pid_y.compute(err_y) / VEL_SCALE
                    vr = -pid_x.compute(err_x) / VEL_SCALE
                    yr = -pid_x.compute(err_x) / YAW_SCALE
            else:
                cv2.putText(frame, "QR LOST – HOLDING", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

            send_body_velocity(vf, vr, vz, yr)

            # End inspection
            if current_time - inspection_start_time > INSPECTION_DURATION:
                print("Inspection complete – Returning to AUTO mode")
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    3  # AUTO
                )
                state = "MONITORING"

        # Draw center
        cv2.circle(frame, (cx, cy), deadzone, (255, 255, 255), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

        cv2.imshow("Hybrid QR Inspection - PX4", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("\nScript stopped.")
    if state == "INSPECTING":
        send_hover()
        time.sleep(1)
    cap.release()
    cv2.destroyAllWindows()
    print("Safe stop. Mission can continue in QGC.")
