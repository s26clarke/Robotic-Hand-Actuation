import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- Load CSV ---
df = pd.read_csv("OpenCV Tracking/datath1.csv")

# Remove NaNs/Infs
df = df[np.isfinite(df["ServoAngle"]) & np.isfinite(df["Theta2"])]

# Aggregate duplicates
df = df.groupby("ServoAngle", as_index=False).mean()

# Sort by servo angle
df = df.sort_values("ServoAngle")

servo = df["ServoAngle"].values
theta2 = df["Theta2"].values

# --- Fit polynomial ---
degree = 10  # adjust degree for smoothness
coeffs = np.polyfit(theta2, servo, degree)


def get_motor_angle(theta2_desired):
    servo_angle = np.polyval(coeffs, theta2_desired)
    return np.clip(servo_angle, 2, 155)  # clip to physical limits

# --- Example usage ---
desired_theta2 = -60
required_servo = get_motor_angle(desired_theta2)
print(f"To achieve Theta2 = {desired_theta2}°, set ServoAngle = {required_servo:.2f}°")

# --- Optional plot ---
theta_fit = np.linspace(theta2.min(), theta2.max(), 500)
servo_fit = np.polyval(coeffs, theta_fit)

plt.plot(theta_fit, servo_fit, 'r-', label=f'{degree}th-degree fit')
plt.scatter(theta2, servo, s=10, label='Data')
plt.xlabel('Theta2 (deg)')
plt.ylabel('Servo Angle (deg)')
plt.legend()
plt.show()


# Print polynomial
poly_str = " + ".join([f"{c:.8g}*theta2**{degree-i}" for i, c in enumerate(coeffs)])
print("Polynomial mapping Theta2 -> ServoAngle:")
print(f"servo_angle = {poly_str}")

# Optional: print as array for direct use
print("\nCoefficients array:")
print(coeffs)

