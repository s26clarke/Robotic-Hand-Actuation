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
SERIAL_PORT = 'COM4'
BAUD_RATE = 9600

frames_required = 5
max_distance = 20
tracked_yellow = {}
joint_id = 0

servo_angle = 2
servo_direction = 1   # 1 = up, -1 = down
servo_step = 1        # degrees per sweep step
servo_delay = 0       # seconds between commands

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

def send_servo_angles(front_angle, rear_angle):
    """Send two servo angles to Arduino and return confirmed tuple."""
    try:
        cmd = f"{front_angle},{rear_angle}\n"
        arduino.write(cmd.encode())
        time.sleep(0.1)
        response = arduino.readline().decode().strip()
        if "," in response:
            f_str, r_str = response.split(",")
            return int(f_str), int(r_str)
        return None
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
    # YELLOW JOINT DETECTION (Base + J2)
    # -------------------------------
    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_mask = cv2.medianBlur(yellow_mask, 7)
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yellow_centers = []

    for cnt in yellow_contours:
        area = cv2.contourArea(cnt)
        if 30 < area < 800:
            (x, y), r = cv2.minEnclosingCircle(cnt)
            yellow_centers.append((int(x), int(y), int(r)))

    # -------------------------------
    # GREEN JOINT DETECTION (J3)
    # -------------------------------
    lower_green = np.array([45, 100, 100], dtype=np.uint8)
    upper_green = np.array([90, 255, 255], dtype=np.uint8)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    green_mask = cv2.medianBlur(green_mask, 7)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_centers = []
    for cnt in green_contours:
        area = cv2.contourArea(cnt)
        if 30 < area < 800:
            (x, y), r = cv2.minEnclosingCircle(cnt)
            green_centers.append((int(x), int(y), int(r)))

    # -------------------------------
    # BLUE TIP DETECTION (Tip)
    # -------------------------------
    lower_blue = np.array([100, 150, 100], dtype=np.uint8)
    upper_blue = np.array([130, 255, 255], dtype=np.uint8)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = cv2.medianBlur(blue_mask, 7)
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_centers = []
    for cnt in blue_contours:
        area = cv2.contourArea(cnt)
        if 30 < area < 800:
            (x, y), r = cv2.minEnclosingCircle(cnt)
            blue_centers.append((int(x), int(y), int(r)))

    # -------------------------------
    # DRAW ALL DETECTED JOINTS
    # -------------------------------
    for (x, y, r) in yellow_centers:
        cv2.circle(frame, (x, y), 6, (0, 255, 255), -1)  # yellow
    for (x, y, r) in green_centers:
        cv2.circle(frame, (x, y), 6, (0, 255, 0), -1)  # green
    for (x, y, r) in blue_centers:
        cv2.circle(frame, (x, y), 6, (255, 0, 0), -1)  # blue

    # -------------------------------
    # COMPUTE LINK ANGLES
    # -------------------------------
    if len(yellow_centers) >= 2 and len(green_centers) >= 1 and len(blue_centers) >= 1:
        yellow_sorted = sorted(yellow_centers, key=lambda c: c[0], reverse=True)
        BASE, J2 = [p[:2] for p in yellow_sorted[:2]]
        J3 = green_centers[0][:2]
        TIP = blue_centers[0][:2]

        # --- CALIBRATION PHASE ---
        if L1_nom is None:
            calibration_frames.append([BASE, J2, J3, TIP])
            if len(calibration_frames) >= calibration_required:
                L1_list, L2_list, L3_list = [], [], []
                for pts in calibration_frames:
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

        # Compute vectors
        v1 = vector(BASE, J2)
        v2 = vector(J2, J3)
        v3 = vector(J3, TIP)

        # Compute angles
        theta1 = angle_from_horizontal_signed(v1)
        theta2 = relative_angle(v1, v2)
        theta3 = relative_angle(v2, v3)

        # Smooth angles
        alpha_theta = 0.1
        if len(angle_data) > 0:
            prev = angle_data[-1][1:4]
            theta1 = prev[0]*(1-alpha_theta) + theta1*alpha_theta
            theta2 = prev[1]*(1-alpha_theta) + theta2*alpha_theta
            theta3 = prev[2]*(1-alpha_theta) + theta3*alpha_theta

        # Draw finger links
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
    if frame_count % 15 == 0:
        servo_angle += servo_direction * servo_step
        if servo_angle >= 155 or servo_angle <= 2: #155
            servo_direction *= -1
        rear_angle = 155 - servo_angle
        actual = send_servo_angles(servo_angle, rear_angle)
        if actual is not None:
            servo_angle, rear_angle = actual
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
