#pragma once

#include <string>
#include <systemc>

struct SimConfig {
    std::string case_name       = "normal";
    std::string output_dir      = "results";
    std::string disturbance_csv = "resources/disturbances.csv";

    sc_core::sc_time simulation_duration = sc_core::sc_time(10, sc_core::SC_SEC);

    // ── Timing ────────────────────────────────────────────────────────────
    sc_core::sc_time plant_dt       = sc_core::sc_time(500, sc_core::SC_US);  // 2 kHz RK4
    sc_core::sc_time imu_period     = sc_core::sc_time(1,   sc_core::SC_MS);  // 1 kHz
    sc_core::sc_time encoder_period = sc_core::sc_time(10,  sc_core::SC_MS);  // 100 Hz
    sc_core::sc_time control_period = sc_core::sc_time(1,   sc_core::SC_MS);  // 1 kHz

    // ── FIFO depths ───────────────────────────────────────────────────────
    int imu_fifo_depth         = 16;
    int encoder_fifo_depth     = 16;
    int control_fifo_depth     = 16;
    int disturbance_fifo_depth = 16;

    // ── PID: angle stabilisation (θ → setpoint) ────────────────────────────
    double Kp_theta               = 50.0;
    double Ki_theta               = 5.0;
    double Kd_theta               = 8.0;
    double integrator_clamp_theta = 1.5;

    // ── Outer cascade loop: z position → θ setpoint ─────────────────────────
    // Gains produce a desired angle offset (rad), not force directly.
    // θ_setpoint = Kp_z·z + Ki_z·∫z + Kd_z·ż, clamped to ±theta_setpoint_clamp.
    // Positive z (cart right) → positive θ_setpoint (lean left) → net leftward force.
    double Kp_z                 = 0.075;    // rad/m
    double Ki_z                 = 0.005;   // rad/(m·s)
    double Kd_z                 = 0.075;    // rad·s/m
    double integrator_clamp_z   = 5.0;   // m·s  (raw integral safety clamp)
    double theta_setpoint_clamp = 0.25;   // rad  (~14°, hard limit on angle bias)

    // Asymmetric decay on the z integrator accumulator:
    //   - when the new increment opposes the accumulator (unwinding), scale by
    //     z_accum_decay_factor so it unwinds faster than it builds
    //   - when |F| is small (system settled), exponentially bleed the accumulator
    double z_accum_decay_factor  = 1.5;   // unwind multiplier (> 1.0 = faster decay)
    double z_accum_bleed_rate    = 0.02;  // fraction of int_z bled per second when settled
    double force_decay_threshold = 1.0;   // N — |F| below this triggers bleed

    double force_saturation = 30.0;  // N

    // ── Complementary filter time constants ──────────────────────────────
    // alpha is derived each tick as tau/(tau+dt) so bandwidth is dt-invariant.
    double tau_theta = 0.50;   // gyro/accel crossover (s) — 500ms
    double tau_z     = 0.10;   // IMU/encoder crossover (s) — 100ms
    double tau_zdot  = 0.30;   // encoder finite-diff → z_dot_est anchor (s) — 300ms
    double tau_ddot  = 0.02;   // low-pass for theta_ddot_est and z_ddot_est (s) — 20ms

    // ── Sensor noise (std dev) ────────────────────────────────────────────
    double gyro_noise_std     = 0.15;  // rad/s
    double accel_noise_std    = 0.5;    // m/s²
    double encoder_resolution = 0.1;  // m per encoder count
    
    // ── Compute time distributions (μ, σ) in microseconds ─────────────────
    // Each module samples N(mean, std) each cycle; clamped to ≥ 0
    double imu_compute_mean_us  = 50.0;
    double imu_compute_std_us   = 10.0;

    double enc_compute_mean_us  = 80.0;
    double enc_compute_std_us   = 15.0;

    double fc_compute_mean_us   = 200.0;   // FusionControl normal
    double fc_compute_std_us    = 40.0;
    double fc_disturbed_mean_us = 400.0;   // FusionControl under disturbance
    double fc_disturbed_std_us  = 80.0;

    double plant_compute_mean_us = 100.0;
    double plant_compute_std_us  = 20.0;

    // ── Factory methods ───────────────────────────────────────────────────
    static SimConfig normal() {
        SimConfig cfg;
        cfg.case_name = "normal";
        return cfg;
    }
};
