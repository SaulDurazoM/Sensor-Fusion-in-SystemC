"""
Inverted Pendulum - Combined Plant + Controller Visualizer
Overlays true plant states with controller estimates and internals.
Two CSVs at different sample rates, aligned by timestamp.
"""

from pathlib import Path

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ========== LOAD DATA ==========

RESULTS_DIR = Path("results/normal")

# Set to a filename like "pendulum.mp4" to export the animation instead of showing it
# live. Requires ffmpeg on PATH. Set to None to just display interactively.
SAVE_ANIMATION_TO = "pendulum.mp4"  # e.g. "pendulum.mp4" or "pendulum.gif"
ANIMATION_SPEED = 4.0  # 1.0 = real-time, 0.5 = 2x speed (matches original .m), 2.0 = half speed

P = pd.read_csv(RESULTS_DIR / "plant_state.csv")
C = pd.read_csv(RESULTS_DIR / "control.csv")
IMU = pd.read_csv(RESULTS_DIR / "imu.csv")
ENC = pd.read_csv(RESULTS_DIR / "encoder.csv")

# Plant signals
pt = P["time_s"].to_numpy()
p_theta = P["theta"].to_numpy()
p_theta_d = P["theta_dot"].to_numpy()
p_theta_dd = P["theta_ddot"].to_numpy()
p_z = P["z"].to_numpy()
p_z_d = P["z_dot"].to_numpy()
p_z_dd = P["z_ddot"].to_numpy()
p_F = P["force_applied"].to_numpy()
p_tau = P["tau_disturbance"].to_numpy()

# Controller signals
ct = C["time_s"].to_numpy()
c_theta_est_accel = C["theta_est_accel"].to_numpy()
c_theta_est = C["theta_est"].to_numpy()
c_theta_d_est = C["theta_dot_est"].to_numpy()
c_theta_dd_est = C["theta_ddot_est"].to_numpy()
c_z_est = C["z_est"].to_numpy()
c_z_d_est = C["z_dot_est"].to_numpy()
c_z_dd_est = C["z_ddot_est"].to_numpy()
c_outer_P = C["outer_P"].to_numpy()
c_outer_I = C["outer_I"].to_numpy()
c_outer_D = C["outer_D"].to_numpy()
c_sp_raw = C["theta_setpoint_raw"].to_numpy()
c_sp = C["theta_setpoint"].to_numpy()
c_e_theta = C["e_theta"].to_numpy()
c_inner_P = C["inner_P"].to_numpy()
c_inner_I = C["inner_I"].to_numpy()
c_inner_D = C["inner_D"].to_numpy()
c_F_raw = C["force_raw"].to_numpy()
c_F = C["force"].to_numpy()

# IMU signals
it = IMU["time_s"].to_numpy()
i_omega = IMU["omega"].to_numpy()
i_ax = IMU["a_x_prime"].to_numpy()
i_ay = IMU["a_y_prime"].to_numpy()
i_dist = IMU["disturbed"].to_numpy()

# Encoder signals
et = ENC["time_s"].to_numpy()
e_z = ENC["z_quantized"].to_numpy()

# Pendulum length — must match simulation
L = 0.5


def interp_nan(x, xp, fp):
    """Linear interp, NaN outside the original range (matches MATLAB interp1 default)."""
    out = np.interp(x, xp, fp)
    out[(x < xp[0]) | (x > xp[-1])] = np.nan
    return out


def interp_previous(x, xp, fp):
    """Zero-order hold (previous-value) interpolation, NaN outside."""
    idx = np.searchsorted(xp, x, side="right") - 1
    out = np.where(idx >= 0, fp[np.clip(idx, 0, len(fp) - 1)], np.nan)
    out[(x < xp[0]) | (x > xp[-1])] = np.nan
    return out


# ========== FIGURE 1: STATE COMPARISONS ==========

fig1, axes1 = plt.subplots(3, 2, figsize=(13, 9))
fig1.canvas.manager.set_window_title("States: True vs Estimated")

# --- Theta ---
ax = axes1[0, 0]
ax.plot(pt, np.rad2deg(np.mod(p_theta + np.pi, 2 * np.pi) - np.pi), "b", linewidth=1.2, label="True")
ax.plot(ct, np.rad2deg(c_theta_est), "r--", linewidth=1.0, label="Estimated")
ax.plot(ct, np.rad2deg(c_theta_est_accel), "g-.", linewidth=0.8, label="Estimated (accel only)")
ax.plot(ct, np.rad2deg(c_sp), color=(0.5, 0.5, 0), linewidth=0.8, label="Setpoint (clamped)")
ax.plot(ct, np.rad2deg(c_sp_raw), ":", color=(0.5, 0.5, 0), linewidth=0.8, label="Setpoint (raw)")
ax.axhline(0, color="k", linestyle="--", linewidth=0.5)
ax.set_xlabel("Time [s]"); ax.set_ylabel("Angle [deg]")
ax.set_title(r"$\theta$: True vs Estimated vs Setpoint")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Theta dot ---
ax = axes1[0, 1]
ax.plot(pt, np.rad2deg(p_theta_d), "b", linewidth=1.2, label="True")
ax.plot(ct, np.rad2deg(c_theta_d_est), "r--", linewidth=1.0, label="Estimated")
ax.set_xlabel("Time [s]"); ax.set_ylabel("Angular Velocity [deg/s]")
ax.set_title(r"$\dot{\theta}$: True vs Estimated")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Z ---
ax = axes1[1, 0]
ax.plot(pt, p_z, "b", linewidth=1.2, label="True")
ax.plot(ct, c_z_est, "r--", linewidth=1.0, label="Estimated")
ax.axhline(0, color="k", linestyle="--", linewidth=0.5)
ax.set_xlabel("Time [s]"); ax.set_ylabel("Position [m]")
ax.set_title("z: True vs Estimated")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Z dot ---
ax = axes1[1, 1]
ax.plot(pt, p_z_d, "b", linewidth=1.2, label="True")
ax.plot(ct, c_z_d_est, "r--", linewidth=1.0, label="Estimated")
ax.set_xlabel("Time [s]"); ax.set_ylabel("Velocity [m/s]")
ax.set_title(r"$\dot{z}$: True vs Estimated")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Theta ddot ---
ax = axes1[2, 0]
ax.plot(pt, p_theta_dd, "b", linewidth=1.2, label="True")
ax.plot(ct, c_theta_dd_est, "r--", linewidth=1.0, label="Estimated")
ax.set_xlabel("Time [s]"); ax.set_ylabel(r"Angular Accel [rad/s$^2$]")
ax.set_title(r"$\ddot{\theta}$: True vs Estimated")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Z ddot ---
ax = axes1[2, 1]
ax.plot(pt, p_z_dd, "b", linewidth=1.2, label="True")
ax.plot(ct, c_z_dd_est, "r--", linewidth=1.0, label="Estimated")
ax.set_xlabel("Time [s]"); ax.set_ylabel(r"Accel [m/s$^2$]")
ax.set_title(r"$\ddot{z}$: True vs Estimated")
ax.legend(loc="best", fontsize=7); ax.grid(True)

fig1.suptitle("True Plant States vs Controller Estimates", fontsize=14, fontweight="bold")
fig1.tight_layout()

# ========== FIGURE 2: ESTIMATION ERRORS ==========

p_theta_at_ct    = interp_nan(ct, pt, p_theta)
p_theta_d_at_ct  = interp_nan(ct, pt, p_theta_d)
p_theta_dd_at_ct = interp_nan(ct, pt, p_theta_dd)
p_z_at_ct        = interp_nan(ct, pt, p_z)
p_z_d_at_ct      = interp_nan(ct, pt, p_z_d)
p_z_dd_at_ct     = interp_nan(ct, pt, p_z_dd)

fig2, axes2 = plt.subplots(3, 2, figsize=(12, 9))
fig2.canvas.manager.set_window_title("Estimation Errors")

err_color = (0.8, 0, 0)

ax = axes2[0, 0]
ax.plot(ct, np.rad2deg(p_theta_at_ct - c_theta_est), color=err_color, linewidth=1.0)
ax.set_xlabel("Time [s]"); ax.set_ylabel("Error [deg]")
ax.set_title(r"$\theta$ Estimation Error (true - est)"); ax.grid(True)

ax = axes2[0, 1]
ax.plot(ct, np.rad2deg(p_theta_d_at_ct - c_theta_d_est), color=err_color, linewidth=1.0)
ax.set_xlabel("Time [s]"); ax.set_ylabel("Error [deg/s]")
ax.set_title(r"$\dot{\theta}$ Estimation Error"); ax.grid(True)

ax = axes2[1, 0]
ax.plot(ct, p_z_at_ct - c_z_est, color=err_color, linewidth=1.0)
ax.set_xlabel("Time [s]"); ax.set_ylabel("Error [m]")
ax.set_title("z Estimation Error"); ax.grid(True)

ax = axes2[1, 1]
ax.plot(ct, p_z_d_at_ct - c_z_d_est, color=err_color, linewidth=1.0)
ax.set_xlabel("Time [s]"); ax.set_ylabel("Error [m/s]")
ax.set_title(r"$\dot{z}$ Estimation Error"); ax.grid(True)

ax = axes2[2, 0]
ax.plot(ct, p_theta_dd_at_ct - c_theta_dd_est, color=err_color, linewidth=1.0)
ax.set_xlabel("Time [s]"); ax.set_ylabel(r"Error [rad/s$^2$]")
ax.set_title(r"$\ddot{\theta}$ Estimation Error"); ax.grid(True)

ax = axes2[2, 1]
ax.plot(ct, p_z_dd_at_ct - c_z_dd_est, color=err_color, linewidth=1.0)
ax.set_xlabel("Time [s]"); ax.set_ylabel(r"Error [m/s$^2$]")
ax.set_title(r"$\ddot{z}$ Estimation Error"); ax.grid(True)

fig2.suptitle("Sensor Fusion Estimation Errors", fontsize=14, fontweight="bold")
fig2.tight_layout()

# ========== FIGURE 3: CONTROLLER INTERNALS ==========

fig3, axes3 = plt.subplots(3, 2, figsize=(13, 9))
fig3.canvas.manager.set_window_title("Controller Internals")

# --- Outer loop PID contributions ---
ax = axes3[0, 0]
ax.plot(ct, c_outer_P, "b", linewidth=1.0, label="P")
ax.plot(ct, c_outer_I, color=(0, 0.6, 0), linewidth=1.0, label="I")
ax.plot(ct, c_outer_D, "r", linewidth=1.0, label="D")
ax.set_xlabel("Time [s]"); ax.set_ylabel("[rad]")
ax.set_title(r"Outer Loop (z $\rightarrow$ $\theta_{sp}$): PID Terms")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Theta setpoint raw vs clamped ---
ax = axes3[0, 1]
ax.plot(ct, np.rad2deg(c_sp_raw), color=(0.7, 0.5, 0), linewidth=1.0, label="Raw")
ax.plot(ct, np.rad2deg(c_sp), color=(0, 0, 0.8), linewidth=1.2, label="Clamped")
ax.set_xlabel("Time [s]"); ax.set_ylabel("Angle [deg]")
ax.set_title(r"$\theta$ Setpoint: Raw vs Clamped")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Inner loop error ---
ax = axes3[1, 0]
ax.plot(ct, np.rad2deg(c_e_theta), "b", linewidth=1.0)
ax.axhline(0, color="k", linestyle="--", linewidth=0.5)
ax.set_xlabel("Time [s]"); ax.set_ylabel("Error [deg]")
ax.set_title(r"Inner Loop: $\theta$ Error (setpoint - est)"); ax.grid(True)

# --- Inner loop PID contributions ---
ax = axes3[1, 1]
ax.plot(ct, c_inner_P, "b", linewidth=1.0, label="P")
ax.plot(ct, c_inner_I, color=(0, 0.6, 0), linewidth=1.0, label="I")
ax.plot(ct, c_inner_D, "r", linewidth=1.0, label="D")
ax.set_xlabel("Time [s]"); ax.set_ylabel("[N]")
ax.set_title(r"Inner Loop ($\theta \rightarrow F$): PID Terms")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Force raw vs saturated ---
ax = axes3[2, 0]
ax.plot(ct, c_F_raw, color=(0.7, 0, 0), linewidth=1.0, label="Raw (unbounded)")
ax.plot(ct, c_F, "b", linewidth=1.2, label="Applied (saturated)")
ax.set_xlabel("Time [s]"); ax.set_ylabel("Force [N]")
ax.set_title("Force: Raw vs Saturated")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Disturbance torque (stem) ---
ax = axes3[2, 1]
ax.stem(pt, p_tau, linefmt="-", markerfmt=" ", basefmt=" ")
for line in ax.lines:
    line.set_color((0.8, 0, 0.4))
    line.set_linewidth(1.2)
ax.set_xlabel("Time [s]"); ax.set_ylabel(r"Torque [N$\cdot$m]")
ax.set_title("Disturbance Torque"); ax.grid(True)

fig3.suptitle("Controller Internals: Cascade PID", fontsize=14, fontweight="bold")
fig3.tight_layout()

# ========== FIGURE 4: PHASE PORTRAITS ==========

fig4, axes4 = plt.subplots(1, 2, figsize=(10, 4.5))
fig4.canvas.manager.set_window_title("Phase Portraits")

ax = axes4[0]
ax.plot(np.rad2deg(p_theta), np.rad2deg(p_theta_d), "b", linewidth=0.8, label="Trajectory")
ax.plot(np.rad2deg(p_theta[0]), np.rad2deg(p_theta_d[0]), "go", markersize=10, markeredgewidth=2,
        markerfacecolor="none", label="Start")
ax.plot(np.rad2deg(p_theta[-1]), np.rad2deg(p_theta_d[-1]), "rx", markersize=10, markeredgewidth=2,
        label="End")
ax.set_xlabel("Angle [deg]"); ax.set_ylabel("Angular Velocity [deg/s]")
ax.set_title(r"$\theta$ Phase Portrait")
ax.legend(loc="best"); ax.grid(True)

ax = axes4[1]
ax.plot(p_z, p_z_d, color=(0, 0.6, 0), linewidth=0.8, label="Trajectory")
ax.plot(p_z[0], p_z_d[0], "go", markersize=10, markeredgewidth=2,
        markerfacecolor="none", label="Start")
ax.plot(p_z[-1], p_z_d[-1], "rx", markersize=10, markeredgewidth=2, label="End")
ax.set_xlabel("Position [m]"); ax.set_ylabel("Velocity [m/s]")
ax.set_title("Cart Phase Portrait")
ax.legend(loc="best"); ax.grid(True)

fig4.suptitle("Phase Portraits", fontsize=14, fontweight="bold")
fig4.tight_layout()

# ========== FIGURE 5: SENSOR DATA ==========

# Compute true (noiseless) IMU values from plant for comparison
ip_theta    = interp_nan(it, pt, p_theta)
ip_theta_d  = interp_nan(it, pt, p_theta_d)
ip_theta_dd = interp_nan(it, pt, p_theta_dd)
ip_z_dd     = interp_nan(it, pt, p_z_dd)

# True accelerometer values (same equations as IMUSensor.cpp)
true_ax = -L * ip_theta_dd + 9.81 * np.sin(ip_theta) + ip_z_dd * np.cos(ip_theta)
true_ay = -L * ip_theta_d**2 + 9.81 * np.cos(ip_theta) - ip_z_dd * np.sin(ip_theta)
true_ax_g = 9.81 * np.sin(ip_theta)
true_ay_g = 9.81 * np.cos(ip_theta)

fig5, axes5 = plt.subplots(3, 2, figsize=(13, 9))
fig5.canvas.manager.set_window_title("Sensor Data")

# --- Gyro: omega vs true theta_dot ---
ax = axes5[0, 0]
ax.plot(it, i_omega, ".", color=(0.6, 0.6, 0.6), markersize=2, label="Gyro (noisy)")
ax.plot(pt, p_theta_d, "b", linewidth=1.2, label=r"True $\dot{\theta}$")
ax.set_xlabel("Time [s]"); ax.set_ylabel("[rad/s]")
ax.set_title(r"Gyro: $\omega$ (measured) vs true $\dot{\theta}$")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Gyro noise residual ---
ax = axes5[0, 1]
ax.plot(it, i_omega - ip_theta_d, ".", color=(0.8, 0, 0), markersize=2)
ax.set_xlabel("Time [s]"); ax.set_ylabel("[rad/s]")
ax.set_title("Gyro Noise (measured - true)"); ax.grid(True)

# --- Accelerometer x' ---
ax = axes5[1, 0]
ax.plot(it, i_ax, ".", color=(0.6, 0.6, 0.6), markersize=2, label="Measured")
ax.plot(it, true_ax, "b", linewidth=1.0, label="True")
ax.plot(it, true_ax_g, "r-", linewidth=1.0, label="Gravity Only")
ax.set_xlabel("Time [s]"); ax.set_ylabel(r"[m/s$^2$]")
ax.set_title(r"Accel $a_{x'}$: Measured vs True")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Accelerometer y' ---
ax = axes5[1, 1]
ax.plot(it, i_ay, ".", color=(0.6, 0.6, 0.6), markersize=2, label="Measured")
ax.plot(it, true_ay, "b", linewidth=1.0, label="True")
ax.plot(it, true_ay_g, "r-", linewidth=1.0, label="Gravity Only")
ax.set_xlabel("Time [s]"); ax.set_ylabel(r"[m/s$^2$]")
ax.set_title(r"Accel $a_{y'}$: Measured vs True")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Encoder: z_quantized vs true z ---
ax = axes5[2, 0]
ax.step(et, e_z, where="post", color=(0, 0.5, 0), linewidth=1.0, label="Encoder (quantized)")
ax.plot(pt, p_z, "b", linewidth=1.2, label="True z")
ax.set_xlabel("Time [s]"); ax.set_ylabel("[m]")
ax.set_title(r"Encoder: $z_{quantized}$ vs True z")
ax.legend(loc="best", fontsize=7); ax.grid(True)

# --- Encoder quantization error ---
ax = axes5[2, 1]
ep_z = interp_nan(et, pt, p_z)
ax.step(et, e_z - ep_z, where="post", color=(0.8, 0, 0), linewidth=1.0)
ax.set_xlabel("Time [s]"); ax.set_ylabel("[m]")
ax.set_title("Encoder Quantization Error"); ax.grid(True)

fig5.suptitle("Sensor Data: IMU + Encoder", fontsize=14, fontweight="bold")
fig5.tight_layout()

# ========== ANIMATION ==========

fig6 = plt.figure(figsize=(9, 5.5))
fig6.canvas.manager.set_window_title("Pendulum Animation")
ax6 = fig6.add_subplot(111)

N = len(pt)
target_fps = 30
dt_avg = float(np.mean(np.diff(pt)))
skip = max(1, round(1 / (target_fps * dt_avg)))
idx = np.arange(0, N, skip)

# Interpolate controller setpoint and force onto plant timestamps (zero-order hold)
c_sp_at_pt = interp_previous(pt, ct, c_sp)
c_F_at_pt  = interp_previous(pt, ct, c_F)

cart_w = 0.15
cart_h = 0.08
wheel_r = 0.02
circ = np.linspace(0, 2 * np.pi, 30)


def draw_frame(k):
    ax6.clear()

    z_k = p_z[k]
    th_k = p_theta[k]
    sp_k = c_sp_at_pt[k]
    F_k = c_F_at_pt[k]
    tau_k = p_tau[k]

    # True pendulum tip (CCW-positive)
    pend_x = z_k - L * np.sin(th_k)
    pend_y = L * np.cos(th_k)

    # Rail
    rail_span = 1.0
    ax6.plot([z_k - rail_span, z_k + rail_span], [0, 0], "k-", linewidth=2)

    # Cart
    cart = patches.FancyBboxPatch(
        (z_k - cart_w, -cart_h / 2), 2 * cart_w, cart_h,
        boxstyle="round,pad=0,rounding_size=0.015",
        facecolor=(0.3, 0.5, 0.8), edgecolor="k", linewidth=1.5,
    )
    ax6.add_patch(cart)

    # Wheels
    for wz in (z_k - cart_w * 0.6, z_k + cart_w * 0.6):
        ax6.fill(
            wz + wheel_r * np.cos(circ),
            -cart_h / 2 + wheel_r * np.sin(circ) - wheel_r,
            color=(0.3, 0.3, 0.3),
        )

    # Ghost pendulum at setpoint angle (drawn first so real one is on top)
    if not np.isnan(sp_k):
        ghost_x = z_k - L * np.sin(sp_k)
        ghost_y = L * np.cos(sp_k)
        ax6.plot([z_k, ghost_x], [0, ghost_y], "--",
                 color=(0.4, 0.8, 0.4, 0.6), linewidth=2)
        ax6.plot(ghost_x, ghost_y, "o", markersize=10,
                 markerfacecolor=(0.4, 0.8, 0.4), markeredgecolor="none")

    # True pendulum rod
    ax6.plot([z_k, pend_x], [0, pend_y], color=(0.7, 0.2, 0.2), linewidth=3)

    # Tip mass
    ax6.plot(pend_x, pend_y, "o", markersize=12,
             markerfacecolor=(0.8, 0.1, 0.1), markeredgecolor="k", markeredgewidth=1.5)

    # Pivot
    ax6.plot(z_k, 0, "ko", markersize=6, markerfacecolor="k")

    # Force arrow
    if not np.isnan(F_k) and abs(F_k) > 0.5:
        ax6.quiver(z_k, -cart_h, F_k * 0.01, 0, angles="xy", scale_units="xy", scale=1,
                   color=(0, 0.7, 0), width=0.005)

    # Disturbance indicator
    if abs(tau_k) > 0.01:
        ax6.text(z_k, L + 0.08, rf"$\tau_d$ = {tau_k:.1f}",
                 ha="center", color=(0.8, 0, 0.4), fontsize=11, fontweight="bold")

    sp_deg = "---" if np.isnan(sp_k) else f"{np.rad2deg(sp_k):.2f}"
    ax6.set_title(
        f"t={pt[k]:.3f}s | θ={np.rad2deg(th_k):.2f}° | "
        f"θ_sp={sp_deg}° | z={z_k:.4f}m | F={F_k:.1f}N",
        fontsize=11,
    )

    ax6.set_xlim(z_k - 0.8, z_k + 0.8)
    ax6.set_ylim(-0.3, 0.7)
    ax6.set_aspect("equal")
    ax6.grid(True)


anim_interval_ms = max(1, int(dt_avg * skip * 1000 * ANIMATION_SPEED))
anim = animation.FuncAnimation(
    fig6, draw_frame, frames=idx, interval=anim_interval_ms, repeat=False
)

if SAVE_ANIMATION_TO:
    out_path = Path(SAVE_ANIMATION_TO)
    fps = max(1, int(round(1000 / anim_interval_ms)))
    print(f"Saving animation to {out_path} at {fps} fps ({len(idx)} frames)...")
    if out_path.suffix.lower() == ".gif":
        anim.save(out_path, writer="pillow", fps=fps)
    else:
        anim.save(out_path, writer="ffmpeg", fps=fps, dpi=150,
                  extra_args=["-vcodec", "libx264", "-pix_fmt", "yuv420p"])
    print("Done.")

plt.show()
