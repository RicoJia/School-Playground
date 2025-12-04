# import numpy as np
# import matplotlib.pyplot as plt

# # 1. array factor is a function of angle theta 
# # 2. elements are spaced (at most?) half wavelength apart to make sure signal is sampled sufficiently (Nyquist criterion)?
# # Phasor: a wave cos(ωt+ϕ) = Real_part(exp(j*(ωt+ϕ))). So exp(1j * θ) is a phasor, where its magnitude is 1 and its angle θ


# c = 1500.0  # Speed of sound in water (m/s)
# f = 30000 # Frequency (Hz)
# wave_length = c / f  # Wavelength (m)
# wave_number = 2 * np.pi / wave_length  # Wave number (rad/m)
# num_elements = 8  # Number of transducer elements
# element_spacing = wave_length / 2  # Spacing between elements (m), chosen to be half wavelength to comply with Nyquist criterion
# steer_angle = 30  # Steering angle in degrees

# def get_array_factor(theta_arr, intended_steer_angle):
#     theta_rad = np.deg2rad(theta_arr)
#     steer_rad = np.deg2rad(intended_steer_angle)
#     n = np.arange(num_elements).reshape(-1, 1)  # Element indices from 0 to num_elements-1, reshaped for broadcasting
    
#     # # phase for each element: (sin(theta) - sin(steer_angle)) * wave_number * element_spacing 
#     phase = (np.sin(theta_rad) - np.sin(steer_rad)) * wave_number * element_spacing  * n
#     array_factor_magnitudes = np.abs(np.sum(np.exp(1j * phase), axis=0))
#     array_factor_magnitudes_normalized = array_factor_magnitudes / np.max(array_factor_magnitudes)
#     return array_factor_magnitudes_normalized


# if __name__ == "__main__":
#     theta_arr = np.linspace(-90, 90, 721)  # Angle array from -90 to 90 degrees
#     array_factors = get_array_factor(theta_arr, steer_angle)
#     array_factors_db = 20 * np.log10(array_factors + 1e-12)  

    
#     plt.figure()
#     plt.plot(theta_arr, array_factors_db)
#     plt.xlabel("Angle (degrees)")
#     plt.ylabel("Array factor (dB)")
#     plt.title(f"Beam pattern for N={num_elements}, d=λ/2, steered to {steer_angle}°")
#     plt.ylim(-40, 0)
#     plt.grid(True)
#     plt.show()

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==== Parameters ====
c = 1500.0          # speed of sound (m/s)
f = 30_000.0        # frequency (Hz)
omega = 2 * np.pi * f
lam = c / f         # wavelength
k = 2 * np.pi / lam # wavenumber
N = 8               # number of elements
d = lam / 2         # spacing (half wavelength)

# Element positions along x-axis, centered at 0
x_elements = (np.arange(N) - (N - 1) / 2.0) * d

# Angles we will visualize (degrees)
angles_deg = np.array([-60.0, -30.0, 0.0, 30.0, 60.0])
num_angles = len(angles_deg)

# Time window per angle (seconds) and frames
period = 1.0 / f
cycles_per_angle = 5.0          # ~5 cycles per angle window (visual choice)
T_window = cycles_per_angle * period
frames_per_angle = 40
dt = T_window / frames_per_angle
total_frames = num_angles * frames_per_angle

# Beam pattern for reference (steered to 0°)
theta_scan = np.linspace(-90, 90, 361)

def array_factor(theta_deg, steer_deg):
    theta = np.deg2rad(theta_deg)
    steer = np.deg2rad(steer_deg)
    n = np.arange(N)[:, None]
    phase = k * d * n * (np.sin(theta)[None, :] - np.sin(steer))
    af = np.sum(np.exp(1j * phase), axis=0)
    af_mag = np.abs(af)
    af_mag /= af_mag.max()
    return af_mag

af_mag = array_factor(theta_scan, 0.0)
af_db = 20 * np.log10(np.maximum(af_mag, 1e-6))

# ==== Figure setup ====
fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(11, 4))

# Left: elements as circles (scatter) and total sum as a square
ax_left.set_title("Element signals (circles) and their sum")
ax_left.set_xlabel("Array axis (m)")
ax_left.set_ylabel("Signal amplitude")
ax_left.axhline(0.0)

elem_scat = ax_left.scatter(x_elements, np.zeros_like(x_elements), s=80)  # circles
sum_scat = ax_left.scatter([0.0], [0.0], s=150, marker="s")               # square for sum

y_lim = 1.5
x_margin = d * (N / 2 + 1)
ax_left.set_xlim(x_elements.min() - x_margin, x_elements.max() + x_margin)
ax_left.set_ylim(-y_lim, y_lim)

angle_text = ax_left.text(0.05, 0.9, "", transform=ax_left.transAxes)

# Right: array factor
ax_right.plot(theta_scan, af_db)
marker_pat, = ax_right.plot([], [], marker="o")
ax_right.set_xlabel("Angle (degrees)")
ax_right.set_ylabel("Array factor (dB)")
ax_right.set_title("Array response vs angle (steered to 0°)")
ax_right.set_ylim(-40, 0)
ax_right.grid(True)

def init():
    elem_scat.set_offsets(np.c_[x_elements, np.zeros_like(x_elements)])
    sum_scat.set_offsets([[0.0, 0.0]])
    marker_pat.set_data([], [])
    angle_text.set_text("")
    return elem_scat, sum_scat, marker_pat, angle_text

def update(frame):
    # Which angle and what time within that angle window?
    angle_idx = frame // frames_per_angle
    local_frame = frame % frames_per_angle
    theta_deg = angles_deg[angle_idx]
    theta_rad = np.deg2rad(theta_deg)
    t = local_frame * dt

    # Plane wave arriving from theta_deg (Rx-style)
    phase_n = k * x_elements * np.sin(theta_rad)
    elem_signal = np.sin(omega * t + phase_n)

    # Update element circles
    elem_offsets = np.c_[x_elements, elem_signal]
    elem_scat.set_offsets(elem_offsets)

    # Sum signal shown at x = 0 as a square
    sum_signal = np.mean(elem_signal)
    sum_scat.set_offsets([[0.0, sum_signal]])

    # Update marker on beam pattern
    idx = np.argmin(np.abs(theta_scan - theta_deg))
    marker_pat.set_data([theta_scan[idx]], [af_db[idx]])

    # Text label
    angle_text.set_text(f"Incoming angle: {theta_deg:.0f}°")

    return elem_scat, sum_scat, marker_pat, angle_text

anim = FuncAnimation(
    fig,
    update,
    frames=total_frames,
    init_func=init,
    interval=80,  # ms between frames
    blit=True
)

plt.show()
