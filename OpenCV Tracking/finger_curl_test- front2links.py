import cv2
import numpy as np
import math
from collections import deque
import time
import itertools
import serial

# -------------------------------
# CONFIG
# -------------------------------
USE_BLUE_REF = False
SERIAL_PORT = 'COM4'
BAUD_RATE = 9600

frames_required = 5
max_distance = 20
tracked_joints = {}
joint_id = 0

servo_angle = 2
servo_direction = 1   # 1 = up, -1 = down
servo_step = 1        # degrees per sweep step
servo_delay = 0    # seconds between commands

blue_line_angles = deque(maxlen=10)
log_file = "../finger_angles_with_motor.txt"
angle_data = []
frame_count = 0
start_time = time.time()

# -------------------------------
# AUTO-CALIBRATION SETTINGS
# -------------------------------
calibration_frames = []
calibration_required = 10
L1_nom = L2_nom = L3_nom = None

# -------------------------------
# SERIAL INITIALIZATION
# -------------------------------
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

def send_servo_angle(angle):
    """Send servo angle to Arduino and confirm."""
    try:
        arduino.write(f"{angle}\n".encode())
        time.sleep(0.1)
        response = arduino.readline().decode().strip()
        return int(response) if response.isdigit() else None
    except Exception as e:
        print(f"[SERVO ERROR] {e}")
        return None

# -------------------------------
# UTILITY FUNCTIONS
# -------------------------------
def vector(a, b):
    return np.array([b[0]-a[0], b[1]-a[1]])

def angle_from_horizontal_signed(v):
    return math.degrees(math.atan2(-v[1], v[0]))

def relative_angle(v_from, v_to):
    angle = math.degrees(math.atan2(v_to[1], v_to[0]) - math.atan2(v_from[1], v_from[0]))
    return ((angle + 180) % 360) - 180

def dist(a, b):
    return np.hypot(a[0]-b[0], a[1]-b[1])

# -------------------------------
# VIDEO CAPTURE
# -------------------------------
cap = cv2.VideoCapture(0)

while True:
    frame_count += 1
    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # -------------------------------
    # YELLOW JOINT DETECTION
    # -------------------------------

    # Broader range for yellow detection
    lower_yellow = np.array([20, 80, 80], dtype=np.uint8)
    upper_yellow = np.array([45, 255, 255], dtype=np.uint8)

    #lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    #upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_mask = cv2.medianBlur(yellow_mask, 7)
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    current_centers = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 30 < area < 800:
            (x, y), r = cv2.minEnclosingCircle(cnt)
            current_centers.append((int(x), int(y), int(r)))

    # Track stable joints
    new_tracked = {}
    for cx, cy, r in current_centers:
        matched = False
        for tid, (tx, ty, count) in tracked_joints.items():
            if np.hypot(cx - tx, cy - ty) < max_distance:
                new_tracked[tid] = [cx, cy, count + 1]
                matched = True
                break
        if not matched:
            joint_id += 1
            new_tracked[joint_id] = [cx, cy, 1]
    tracked_joints = new_tracked
    valid_centers = [(x, y) for x, y, count in tracked_joints.values() if count >= frames_required]

    for (x, y) in valid_centers:
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    # -------------------------------
    # COMPUTE LINK ANGLES
    # -------------------------------
    if len(valid_centers) == 4:
        centers = valid_centers

        # --- CALIBRATION PHASE ---
        if L1_nom is None:
            calibration_frames.append(centers)
            if len(calibration_frames) >= calibration_required:
                L1_list, L2_list, L3_list = [], [], []
                for pts in calibration_frames:
                    pts = sorted(pts, key=lambda c: c[0], reverse=True)
                    b, j2, j3, tip = pts
                    L1_list.append(dist(b, j2))
                    L2_list.append(dist(j2, j3))
                    L3_list.append(dist(j3, tip))
                L1_nom, L2_nom, L3_nom = np.mean(L1_list), np.mean(L2_list), np.mean(L3_list)
                print(f"[CALIBRATED] L1={L1_nom:.1f}, L2={L2_nom:.1f}, L3={L3_nom:.1f}")
            else:
                cv2.putText(frame, f"Calibrating... ({len(calibration_frames)}/{calibration_required})",
                            (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow("Finger + Servo Tracking", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue  # wait until calibration done

        # --- GEOMETRIC MATCHING ---
        best_perm, best_error = None, float('inf')
        for perm in itertools.permutations(centers, 4):
            b, j2, j3, tip = perm
            L1, L2, L3 = dist(b, j2), dist(j2, j3), dist(j3, tip)
            err = abs(L1 - L1_nom) + abs(L2 - L2_nom) + abs(L3 - L3_nom)
            if err < best_error:
                best_error = err
                best_perm = perm
        BASE, J2, J3, TIP = best_perm

        # Compute vectors
        v1 = vector(BASE, J2)
        v2 = vector(J2, J3)
        v3 = vector(J3, TIP)

        # Compute angles
        theta1 = angle_from_horizontal_signed(v1)
        theta2 = relative_angle(v1, v2)
        theta3 = relative_angle(v2, v3)

        # Smooth
        alpha_theta = 0.1
        if len(angle_data) > 0:
            prev = angle_data[-1][1:4]
            theta1 = prev[0]*(1-alpha_theta) + theta1*alpha_theta
            theta2 = prev[1]*(1-alpha_theta) + theta2*alpha_theta
            theta3 = prev[2]*(1-alpha_theta) + theta3*alpha_theta

        # Draw links
        cv2.line(frame, BASE, J2, (255, 255, 0), 2)
        cv2.line(frame, J2, J3, (255, 255, 0), 2)
        cv2.line(frame, J3, TIP, (255, 255, 0), 2)
        cv2.putText(frame, f"TH1:{theta1:.1f}", (J2[0] + 10, J2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"TH2:{theta2:.1f}", (J3[0] + 10, J3[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"TH3:{theta3:.1f}", (TIP[0] + 10, TIP[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        timestamp = time.time() - start_time
        angle_data.append((frame_count, theta1, theta2, theta3, servo_angle, timestamp))
        print(f"Frame {frame_count}: Θ1={theta1:.1f}, Θ2={theta2:.1f}, Θ3={theta3:.1f}, Servo={servo_angle}, Time={timestamp:.2f}s")

    # -------------------------------
    # SERVO CONTROL
    # -------------------------------
    if frame_count % 15 == 0:  # every ~0.5s
        servo_angle += servo_direction * servo_step
        if servo_angle >= 155 or servo_angle <= 2:
            servo_direction *= -1
        actual = send_servo_angle(servo_angle)
        if actual is not None:
            servo_angle = actual
        time.sleep(servo_delay)

    # -------------------------------
    # DISPLAY
    # -------------------------------
    cv2.putText(frame, f"Servo: {servo_angle:.0f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Finger + Servo Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# -------------------------------
# SAVE LOG
# -------------------------------
with open(log_file, "w") as f:
    f.write("Frame,Theta1,Theta2,Theta3,ServoAngle,Time(s)\n")
    for row in angle_data:
        f.write(f"{row[0]},{row[1]:.2f},{row[2]:.2f},{row[3]:.2f},{row[4]},{row[5]:.3f}\n")

cap.release()
arduino.close()
cv2.destroyAllWindows()
print(f"Angles + Servo data saved to {log_file}")
