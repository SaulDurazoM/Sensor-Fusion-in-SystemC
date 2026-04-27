#include "top.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>

using namespace sc_core;

// ─────────────────────────────────────────────────────────────────────────────
// IMUSensor
// ─────────────────────────────────────────────────────────────────────────────

IMUSensor::IMUSensor(sc_module_name name,
                     const SimConfig& scfg, const PhysicsConfig& pcfg,
                     const PlantState* state)
    : sc_module(name), scfg_(scfg), pcfg_(pcfg), state_(state),
      gyro_noise_ (0.0, scfg.gyro_noise_std),
      accel_noise_(0.0, scfg.accel_noise_std),
      compute_dist_(scfg.imu_compute_mean_us, scfg.imu_compute_std_us)
{
    SC_THREAD(run);
}

void IMUSensor::run() {
    sc_time next_release = SC_ZERO_TIME;
    while (true) {
        if (sc_time_stamp() < next_release)
            wait(next_release - sc_time_stamp());
        next_release += scfg_.imu_period;

        const double comp = std::max(0.0, compute_dist_(rng_));
        if (comp > 0.0) wait(sc_time(comp, SC_US));

        IMUSample s;
        s.seq       = seq_++;
        s.timestamp = sc_time_stamp();
        s.valid     = true;
        s.disturbed = state_->disturbed;

        s.omega = state_->theta_dot + gyro_noise_(rng_);

        // Raw body-frame accelerometer outputs — FusionControl interprets these.
        // Frame: x̂'=(cosθ, sinθ) aligns with world +x at upright (right-handed with ŷ').
        // a_x' (perp to rod, +x at upright): −L·θ̈ + g·sin θ + z̈·cos θ
        // a_y' (along rod,   +y at upright): −L·θ̇² + g·cos θ − z̈·sin θ
        const double th  = state_->theta;
        const double thd = state_->theta_dot;
        const double zdd = state_->z_ddot;
        const double tdd = state_->theta_ddot;

        s.a_x_prime = -pcfg_.L * tdd
                    + pcfg_.g * std::sin(th)
                    + zdd * std::cos(th)
                    + accel_noise_(rng_);

        s.a_y_prime = -pcfg_.L * thd * thd
                    + pcfg_.g * std::cos(th)
                    - zdd * std::sin(th)
                    + accel_noise_(rng_);

        out.nb_write(s);  // drop on overflow; rate-matched so rarely happens
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// EncoderSensor
// ─────────────────────────────────────────────────────────────────────────────

EncoderSensor::EncoderSensor(sc_module_name name,
                             const SimConfig& scfg, const PlantState* state)
    : sc_module(name), scfg_(scfg), state_(state),
      compute_dist_(scfg.enc_compute_mean_us, scfg.enc_compute_std_us)
{
    SC_THREAD(run);
}

void EncoderSensor::run() {
    sc_time next_release = SC_ZERO_TIME;
    while (true) {
        if (sc_time_stamp() < next_release)
            wait(next_release - sc_time_stamp());
        next_release += scfg_.encoder_period;

        const double comp = std::max(0.0, compute_dist_(rng_));
        if (comp > 0.0) wait(sc_time(comp, SC_US));

        const double res = scfg_.encoder_resolution;

        EncoderSample s;
        s.seq         = seq_++;
        s.timestamp   = sc_time_stamp();
        s.valid       = true;
        s.z_quantized = std::round(state_->z / res) * res;

        out.nb_write(s);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FusionControl
// ─────────────────────────────────────────────────────────────────────────────

FusionControl::FusionControl(sc_module_name name,
                             const SimConfig& scfg, const PhysicsConfig& pcfg)
    : sc_module(name), scfg_(scfg), pcfg_(pcfg),
      compute_normal_dist_  (scfg.fc_compute_mean_us,   scfg.fc_compute_std_us),
      compute_disturbed_dist_(scfg.fc_disturbed_mean_us, scfg.fc_disturbed_std_us)
{
    SC_THREAD(run);
}

double FusionControl::clamp(double v, double limit) const {
    return std::clamp(v, -limit, limit);
}

double FusionControl::wrap_angle(double a) const {
    return std::atan2(std::sin(a), std::cos(a));
}

void FusionControl::run() {
    sc_time next_release = SC_ZERO_TIME;

    while (true) {
        if (sc_time_stamp() < next_release)
            wait(next_release - sc_time_stamp());

        const sc_time release = sc_time_stamp();
        next_release += scfg_.control_period;

        const double dt = first_tick_ ? 0.0 : (release - last_time_).to_seconds();
        last_time_  = release;
        first_tick_ = false;

        // Drain FIFOs into persistent members — last_imu_/last_enc_ survive across ticks
        { IMUSample tmp;     while (imu_in.nb_read(tmp))  last_imu_ = tmp; }
        bool new_enc = false;
        { EncoderSample tmp; while (enc_in.nb_read(tmp)) { last_enc_ = tmp; new_enc = true; } }

        // Stochastic compute latency; last_imu_.disturbed is sticky across ticks
        const bool disturbed = last_imu_.disturbed;
        const double comp = disturbed
            ? std::max(0.0, compute_disturbed_dist_(rng_))
            : std::max(0.0, compute_normal_dist_(rng_));
        if (comp > 0.0) wait(sc_time(comp, SC_US));

        ControlCommand cmd;
        cmd.seq       = seq_++;
        cmd.timestamp = sc_time_stamp();

        if (!last_imu_.valid || !last_enc_.valid || dt <= 0.0) {
            cmd.valid = false;
            out.nb_write(cmd);
            continue;
        }

        // ── Complementary filter: θ ───────────────────────────────────────
        // Remove previous-tick dynamic bias from both axes, then use atan2.
        // atan2 is self-normalising (g cancels in the ratio) and has uniform
        // noise σ_a/g vs asin's σ_a/(g·cosθ) which blows up off-vertical.
        //   a_x' = −L·θ̈ + g·sinθ + z̈·cosθ  →  corrected ≈ g·sinθ
        //   a_y' = −L·θ̇² + g·cosθ − z̈·sinθ  →  corrected ≈ g·cosθ
        const double a_x_corrected = last_imu_.a_x_prime
            + pcfg_.L * theta_ddot_est_
            - z_ddot_est_ * std::cos(theta_est_);

        const double a_y_corrected = last_imu_.a_y_prime
            + pcfg_.L * theta_dot_est_ * theta_dot_est_   // cancel −L·θ̇²
            + z_ddot_est_ * std::sin(theta_est_);         // cancel −z̈·sinθ

        const double theta_from_accel = std::atan2(a_x_corrected, a_y_corrected);

        // Update θ̈ estimate before overwriting theta_dot_est_ (need old value to diff)
        theta_ddot_est_ = (last_imu_.omega - theta_dot_est_) / dt;

        // alpha derived from time constant each tick so bandwidth is dt-invariant
        const double alpha_theta = scfg_.tau_theta / (scfg_.tau_theta + dt);
        theta_est_     = alpha_theta * (theta_est_ + last_imu_.omega * dt)
                       + (1.0 - alpha_theta) * theta_from_accel;
        theta_dot_est_ = last_imu_.omega;

        // ── Complementary filter: z ───────────────────────────────────────
        // Full rearrangement: a_x' = −L·θ̈ + g·sinθ + z̈·cosθ
        //   → z̈ = (a_x' + L·θ̈_est − g·sinθ) / cosθ
        const double cos_th = std::cos(theta_est_);
        z_ddot_est_ = (std::abs(cos_th) > 0.1)
            ? (last_imu_.a_x_prime + pcfg_.L * theta_ddot_est_
               - pcfg_.g * std::sin(theta_est_)) / cos_th
            : 0.0;
        z_dot_est_ += z_ddot_est_ * dt;
        z_est_     += z_dot_est_ * dt;
        if (new_enc && last_enc_.valid) {
            const double alpha_z = scfg_.tau_z / (scfg_.tau_z + dt);
            z_est_ = alpha_z * z_est_ + (1.0 - alpha_z) * last_enc_.z_quantized;
        }

        // ── Outer loop: z position → θ setpoint ──────────────────────────
        // Integrator with asymmetric decay: when the new increment opposes the
        // current accumulator (i.e. we are unwinding), scale it by decay_factor
        // so the accumulator collapses faster than it built up.
        const double z_increment = z_est_ * dt;
        const bool   z_unwinding = (int_z_ * z_increment) < 0.0;
        const double z_decay     = z_unwinding ? scfg_.z_accum_decay_factor : 1.0;
        int_z_ = clamp(int_z_ + z_decay * z_increment, scfg_.integrator_clamp_z);

        double theta_setpoint = scfg_.Kp_z * z_est_
                              + scfg_.Ki_z * int_z_
                              + scfg_.Kd_z * z_dot_est_;
        theta_setpoint = clamp(theta_setpoint, scfg_.theta_setpoint_clamp);

        // ── Inner loop: single angle PID toward θ_setpoint ───────────────
        const double e_theta = wrap_angle(theta_est_ - theta_setpoint);
        int_theta_ = clamp(int_theta_ + e_theta * dt, scfg_.integrator_clamp_theta);
        const double F = -(scfg_.Kp_theta * e_theta
                         + scfg_.Ki_theta * int_theta_
                         + scfg_.Kd_theta * theta_dot_est_);

        cmd.force = clamp(F, scfg_.force_saturation);
        cmd.valid = true;
        out.nb_write(cmd);

        // Bleed z integrator when settled — if |F| is small the system is near
        // equilibrium and any residual accumulation should drain away passively.
        if (std::abs(cmd.force) < scfg_.force_decay_threshold) {
            int_z_ *= (1.0 - scfg_.z_accum_bleed_rate * dt);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Plant
// ─────────────────────────────────────────────────────────────────────────────

Plant::Plant(sc_module_name name,
             const SimConfig& scfg, const PhysicsConfig& pcfg)
    : sc_module(name), scfg_(scfg), pcfg_(pcfg),
      compute_dist_(scfg.plant_compute_mean_us, scfg.plant_compute_std_us)
{
    state_.theta     = pcfg_.theta_0;
    state_.theta_dot = pcfg_.theta_dot_0;
    state_.z         = pcfg_.z_0;
    state_.z_dot     = pcfg_.z_dot_0;

    const std::string dir  = scfg_.output_dir + "/" + scfg_.case_name;
    const std::string path = dir + "/plant_state.csv";
    std::filesystem::create_directories(dir);
    csv_.open(path);
    if (!csv_.is_open())
        SC_REPORT_FATAL("Plant", ("Cannot open output CSV: " + path).c_str());
    write_csv_header();

    SC_THREAD(run);
}

Plant::~Plant() {
    if (csv_.is_open()) csv_.close();
}

void Plant::write_csv_header() {
    csv_ << "time_s,theta,theta_dot,z,z_dot,force_applied,tau_disturbance\n";
}

void Plant::write_csv_row(double t) {
    csv_ << t                      << ","
         << state_.theta           << ","
         << state_.theta_dot       << ","
         << state_.z               << ","
         << state_.z_dot           << ","
         << current_force_         << ","
         << state_.tau_disturbance << "\n";
}

// Nonlinear equations of motion via Euler-Lagrange, CCW-positive θ convention.
//
// Mass matrix M · [z̈; θ̈] = f  (row 0 = z equation, row 1 = θ equation)
// Returns dx/dt = [θ̇, θ̈, ż, z̈]  for state x = [θ, θ̇, z, ż]
Plant::Vec4 Plant::dynamics(const Vec4& x, double F, double tau_d) const {
    const double theta     = x[0];
    const double theta_dot = x[1];
    const double z_dot     = x[3];

    const double mc = pcfg_.m_c;
    const double mp = pcfg_.m_p;
    const double L  = pcfg_.L;
    const double g  = pcfg_.g;
    const double mu = pcfg_.mu;
    const double Ip = pcfg_.I_pivot();

    Eigen::Matrix2d M;
    M(0, 0) = mc + mp;
    M(0, 1) = -0.5 * mp * L * std::cos(theta);
    M(1, 0) = M(0, 1);
    M(1, 1) = Ip;

    // Smoothed Coulomb friction on cart (tanh regularisation avoids sign discontinuity)
    const double f_fric = mu * (mc + mp) * g * std::tanh(z_dot / 0.001);

    Eigen::Vector2d f;
    f[0] = F - f_fric - 0.5 * mp * L * theta_dot * theta_dot * std::sin(theta);
    f[1] = tau_d + mp * g * (L / 2.0) * std::sin(theta);

    const Eigen::Vector2d accel = M.ldlt().solve(f);  // [z̈, θ̈]

    Vec4 dx;
    dx[0] = x[1];       // dθ/dt  = θ̇
    dx[1] = accel[1];   // dθ̇/dt = θ̈
    dx[2] = x[3];       // dz/dt  = ż
    dx[3] = accel[0];   // dż/dt  = z̈
    return dx;
}

Plant::Vec4 Plant::rk4_step(const Vec4& x, double F, double tau_d, double dt) const {
    const Vec4 k1 = dynamics(x,                 F, tau_d);
    const Vec4 k2 = dynamics(x + 0.5*dt*k1,     F, tau_d);
    const Vec4 k3 = dynamics(x + 0.5*dt*k2,     F, tau_d);
    const Vec4 k4 = dynamics(x +     dt*k3,     F, tau_d);
    return x + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

void Plant::run() {
    sc_time next_release = SC_ZERO_TIME;
    while (true) {
        if (sc_time_stamp() < next_release)
            wait(next_release - sc_time_stamp());
        next_release += scfg_.plant_dt;

        const double comp = std::max(0.0, compute_dist_(rng_));
        if (comp > 0.0) wait(sc_time(comp, SC_US));

        // Non-blocking drain: keep latest control force
        { ControlCommand tmp;
          while (control_in.nb_read(tmp))
              if (tmp.valid) current_force_ = tmp.force;
        }

        // Non-blocking drain: keep latest disturbance torque
        { DisturbanceTorque tmp;
          while (disturbance_in.nb_read(tmp))
              state_.tau_disturbance = tmp.tau;
        }
        state_.disturbed = (state_.tau_disturbance != 0.0);

        // RK4 integration step
        const double dt = scfg_.plant_dt.to_seconds();
        Vec4 x(state_.theta, state_.theta_dot, state_.z, state_.z_dot);
        const Vec4 xn = rk4_step(x, current_force_, state_.tau_disturbance, dt);

        state_.theta     = xn[0];
        state_.theta_dot = xn[1];
        state_.z         = xn[2];
        state_.z_dot     = xn[3];

        // Recompute accelerations at new state so IMU reads correct values next tick
        const Vec4 dxn = dynamics(xn, current_force_, state_.tau_disturbance);
        state_.theta_ddot = dxn[1];
        state_.z_ddot     = dxn[3];

        write_csv_row(sc_time_stamp().to_seconds());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Disturbance
// ─────────────────────────────────────────────────────────────────────────────

Disturbance::Disturbance(sc_module_name name,
                         const SimConfig& scfg, const PhysicsConfig& pcfg)
    : sc_module(name), scfg_(scfg), pcfg_(pcfg)
{
    SC_THREAD(run);
}

void Disturbance::run() {
    for (const auto& ev : pcfg_.disturbances) {
        const sc_time t_start = sc_time(ev.time_s,    SC_SEC);
        const sc_time t_dur   = sc_time(ev.duration_s, SC_SEC);

        if (sc_time_stamp() < t_start)
            wait(t_start - sc_time_stamp());

        DisturbanceTorque on;
        on.tau       = ev.torque_nm;
        on.timestamp = sc_time_stamp();
        out.write(on);  // blocking — blocks only if FIFO full (depth 16, never reached)

        wait(t_dur);

        DisturbanceTorque off;
        off.tau       = 0.0;
        off.timestamp = sc_time_stamp();
        out.write(off);
    }

    wait();  // all events exhausted — suspend forever
}

// ─────────────────────────────────────────────────────────────────────────────
// Top
// ─────────────────────────────────────────────────────────────────────────────

Top::Top(sc_module_name name, const SimConfig& s, const PhysicsConfig& p)
    : sc_module(name),
      scfg(s), pcfg(p),
      imu_fifo        (s.imu_fifo_depth),
      enc_fifo        (s.encoder_fifo_depth),
      control_fifo    (s.control_fifo_depth),
      disturbance_fifo(s.disturbance_fifo_depth),
      plant      ("plant",       s, p),
      imu        ("imu",         s, p, plant.get_state()),
      encoder    ("encoder",     s,    plant.get_state()),
      fusion_ctrl("fusion_ctrl", s, p),
      disturbance("disturbance", s, p)
{
    imu.out    (imu_fifo);
    encoder.out(enc_fifo);

    fusion_ctrl.imu_in(imu_fifo);
    fusion_ctrl.enc_in(enc_fifo);
    fusion_ctrl.out   (control_fifo);

    plant.control_in    (control_fifo);
    plant.disturbance_in(disturbance_fifo);

    disturbance.out(disturbance_fifo);
}
