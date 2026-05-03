#pragma once

#include <string>
#include <systemc>

struct SimConfig {
    std::string case_name       = "normal";
    std::string output_dir      = "results";
    std::string disturbance_csv = "resources/disturbances.csv";

    sc_core::sc_time simulation_duration = sc_core::sc_time(20, sc_core::SC_SEC);

    // ── Timing ────────────────────────────────────────────────────────────
    sc_core::sc_time plant_dt       = sc_core::sc_time(500, sc_core::SC_US);  // 2 kHz RK4
    sc_core::sc_time imu_period     = sc_core::sc_time(1,   sc_core::SC_MS);  // 1 kHz
    sc_core::sc_time encoder_period = sc_core::sc_time(10,  sc_core::SC_MS);  // 100 Hz
    sc_core::sc_time control_period = sc_core::sc_time(1,   sc_core::SC_MS);  // 1 kHz
    sc_core::sc_time telemetry_period = sc_core::sc_time(10, sc_core::SC_MS); // 100 Hz FIFO snapshots

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
    double Kp_z                 = 0.075;   // rad/m
    double Ki_z                 = 0.005;   // rad/(m·s)
    double Kd_z                 = 0.075;   // rad·s/m
    double integrator_clamp_z   = 5.0;     // m·s  (raw integral safety clamp)
    double theta_setpoint_clamp = 0.25;    // rad  (~14°, hard limit on angle bias)

    // Asymmetric decay on the z integrator accumulator:
    //   - when the new increment opposes the accumulator (unwinding), scale by
    //     z_accum_decay_factor so it unwinds faster than it builds
    //   - when |F| is small (system settled), exponentially bleed the accumulator
    double z_accum_decay_factor  = 1.5;    // unwind multiplier (> 1.0 = faster decay)
    double z_accum_bleed_rate    = 0.02;   // fraction of int_z bled per second when settled
    double force_decay_threshold = 1.0;    // N — |F| below this triggers bleed

    double force_saturation = 30.0;        // N

    // ── Complementary filter time constants ──────────────────────────────
    // alpha is derived each tick as tau/(tau+dt) so bandwidth is dt-invariant.
    double tau_theta = 0.50;   // gyro/accel crossover (s) — 500ms
    double tau_z     = 0.10;   // IMU/encoder crossover (s) — 100ms
    double tau_zdot  = 0.30;   // encoder finite-diff → z_dot_est anchor (s) — 300ms
    double tau_ddot  = 0.02;   // low-pass for theta_ddot_est and z_ddot_est (s) — 20ms

    // ── Sensor noise (std dev) ────────────────────────────────────────────
    double gyro_noise_std     = 0.15;   // rad/s
    double accel_noise_std    = 0.5;    // m/s²
    double encoder_resolution = 0.1;    // m per encoder count

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

    // ── Debug switch (was a #define) ──────────────────────────────────────
    // When true, FusionControl bypasses estimation and reads PlantState
    // directly. Useful for verifying PID gains in isolation from sensor noise
    // and filter dynamics.
    bool use_full_state = false;

    // ── Factory methods ───────────────────────────────────────────────────
    static SimConfig normal() {
        SimConfig cfg;
        cfg.case_name = "normal";
        return cfg;
    }

    // CPU-stress case: heavy and high-variance compute times, especially in
    // FusionControl when disturbed (mean 1.1 ms exceeds the 1 ms control
    // period, so deadline misses are guaranteed during disturbance windows).
    // Plant compute also climbs toward plant_dt (500 µs) to provoke plant-side
    // misses under the worst draws.
    static SimConfig stress() {
        SimConfig cfg;
        cfg.case_name = "stress";

        cfg.fc_compute_mean_us    = 600.0;   // up from 200
        cfg.fc_compute_std_us     = 200.0;   // up from 40
        cfg.fc_disturbed_mean_us  = 1100.0;  // up from 400 — exceeds 1 ms period
        cfg.fc_disturbed_std_us   = 250.0;   // up from 80

        cfg.plant_compute_mean_us = 350.0;   // up from 100
        cfg.plant_compute_std_us  = 80.0;    // up from 20

        cfg.imu_compute_mean_us   = 150.0;   // up from 50
        cfg.imu_compute_std_us    = 40.0;    // up from 10

        cfg.enc_compute_mean_us   = 200.0;   // up from 80
        cfg.enc_compute_std_us    = 60.0;    // up from 15

        return cfg;
    }

    // Communication-burst case: faster sensor rates and shallow FIFOs so any
    // momentary backpressure from FusionControl produces drops. Compute times
    // are kept at their nominal values — this isolates FIFO/communication
    // stress from CPU stress.
    static SimConfig burst() {
        SimConfig cfg;
        cfg.case_name = "burst";

        cfg.imu_period     = sc_core::sc_time(500, sc_core::SC_US);   // 2 kHz (was 1 kHz)
        cfg.encoder_period = sc_core::sc_time(5,   sc_core::SC_MS);   // 200 Hz (was 100 Hz)

        cfg.imu_fifo_depth     = 4;   // was 16
        cfg.encoder_fifo_depth = 4;   // was 16
        cfg.control_fifo_depth = 4;   // was 16

        return cfg;
    }
};
