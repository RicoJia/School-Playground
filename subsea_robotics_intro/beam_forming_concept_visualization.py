import numpy as np
import matplotlib.pyplot as plt

c = 1500.0
f = 30000
wave_length = c / f
wave_number = 2 * np.pi / wave_length
num_elements = 8
element_spacing = wave_length / 2
steer_angle = 30  # degrees

def get_array_factor(theta_arr, intended_steer_angle):
    theta_rad = np.deg2rad(theta_arr)
    steer_rad = np.deg2rad(intended_steer_angle)
    n = np.arange(num_elements).reshape(-1, 1)
    phase = (np.sin(theta_rad) - np.sin(steer_rad)) * wave_number * element_spacing * n
    array_factor_magnitudes = np.abs(np.sum(np.exp(1j * phase), axis=0))
    return array_factor_magnitudes / np.max(array_factor_magnitudes)

theta_arr = np.linspace(-90, 90, 721)
array_factors = get_array_factor(theta_arr, steer_angle)
array_factors_db = 20 * np.log10(array_factors + 1e-12)

fig, ax = plt.subplots(figsize=(8, 4))
ax.plot(theta_arr, array_factors_db)
ax.axvline(steer_angle, linestyle="--")

ax.set_xlabel("Angle θ (degrees)")
ax.set_ylabel("Array factor (dB, normalized)")
ax.set_title(f"Beam Pattern for {num_elements}-Element Linear Array (Steered to {steer_angle}°)")
ax.set_xlim(-90, 90)
ax.set_ylim(-40, 0)

ax.annotate(
    "Main lobe\n(steer angle)",
    xy=(steer_angle, 0),
    xytext=(steer_angle + 10, -5),
    arrowprops=dict(arrowstyle="->")
)

fig.tight_layout()
plt.show()
