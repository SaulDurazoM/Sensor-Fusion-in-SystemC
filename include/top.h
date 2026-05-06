#pragma once

#include <cstdint>
#include <fstream>
#include <random>
#include <systemc>

#include <Eigen/Dense>

#include "physics_config.h"
#include "sim_config.h"
#include "types.h"

// ─────────────────────────────────────────────────────────────────────────────
// IMUSensor  — 1 kHz, reads shared PlantState, adds Gaussian noise
// ─────────────────────────────────────────────────────────────────────────────
SC_MODULE(IMUSensor) {
public:
    sc_core::sc_fifo_out<IMUSample> out;

    // Counters exposed to Telemetry — written only by this module's thread.
    std::uint64_t emitted_   = 0;
    std::uint64_t drops_     = 0;   // increments when out.nb_write() returns false

    SC_HAS_PROCESS(IMUSensor);
    IMUSensor(sc_core::sc_module_name name,
              const SimConfig& scfg, const PhysicsConfig& pcfg,
              const PlantState* state);
    void run();

private:
    SimConfig     scfg_;
    PhysicsConfig pcfg_;
    const PlantState* state_;
    std::uint64_t seq_ = 0;
    std::mt19937  rng_;
    std::normal_distribution<double> gyro_noise_;
    std::normal_distribution<double> accel_noise_;
    std::normal_distribution<double> compute_dist_;
    std::ofstream csv_;
};

// ─────────────────────────────────────────────────────────────────────────────
// EncoderSensor  — 100 Hz, quantises cart position to encoder_resolution
// ─────────────────────────────────────────────────────────────────────────────
SC_MODULE(EncoderSensor) {
public:
    sc_core::sc_fifo_out<EncoderSample> out;

    std::uint64_t emitted_ = 0;
    std::uint64_t drops_   = 0;

    SC_HAS_PROCESS(EncoderSensor);
    EncoderSensor(sc_core::sc_module_name name,
                  const SimConfig& scfg, const PlantState* state);
    void run();

private:
    SimConfig     scfg_;
    const PlantState* state_;
    std::uint64_t seq_ = 0;
    std::mt19937  rng_;
    std::normal_distribution<double> compute_dist_;
    std::ofstream csv_;
};

// ─────────────────────────────────────────────────────────────────────────────
// FusionControl  — complementary filter + dual PID, runs at control_period
// ─────────────────────────────────────────────────────────────────────────────
SC_MODULE(FusionControl) {
public:
    sc_core::sc_fifo_in<IMUSample>       imu_in;
    sc_core::sc_fifo_in<EncoderSample>   enc_in;
    sc_core::sc_fifo_out<ControlCommand> out;

    // Counters exposed to Telemetry.
    std::uint64_t emitted_          = 0;
    std::uint64_t drops_            = 0;   // out.nb_write() returned false
    std::uint64_t deadline_misses_  = 0;   // tick released later than scheduled

    SC_HAS_PROCESS(FusionControl);
    FusionControl(sc_core::sc_module_name name,
                  const SimConfig& scfg, const PhysicsConfig& pcfg, const PlantState* state);
    void run();

private:
    SimConfig     scfg_;
    PhysicsConfig pcfg_;
    std::uint64_t seq_ = 0;
    std::mt19937  rng_;
    std::normal_distribution<double> compute_normal_dist_;
    std::normal_distribution<double> compute_disturbed_dist_;

    // Latest sensor readings — persist across ticks so filter always has data
    IMUSample     last_imu_;
    EncoderSample last_enc_;
    EncoderSample last_enc_prev_;  // previous encoder sample for z_dot finite difference

    double last_cmd_force_ = 0.0;  // force from previous tick; used for model-based θ̈

    // Used when scfg_.use_full_state is true — bypasses estimation for PID tuning.
    const PlantState* state_;

    // Complementary filter state
    double gyro_ema_        = 0.0;   // EMA-filtered gyro, alpha=0.5 (~3-sample window)
    double theta_est_       = 0.0;
    double theta_from_accel = 0.0;   // estimated angle just from accelerometer
    double a_x_corrected    = 0.0;
    double a_y_corrected    = 0.0;   // overridden in ctor to pcfg.g for a smoother startup
    double theta_dot_est_   = 0.0;
    double theta_ddot_est_  = 0.0;   // model-based (EOM solve); used next tick for accel bias removal
    double z_est_           = 0.0;
    double z_dot_est_       = 0.0;
    double z_ddot_est_      = 0.0;   // stored each tick for next tick's theta bias removal

    // PID integrators
    double int_theta_ = 0.0;
    double int_z_     = 0.0;

    bool             first_tick_ = true;
    sc_core::sc_time last_time_  = sc_core::SC_ZERO_TIME;

    std::ofstream ctrl_csv_;

    double clamp(double v, double limit) const;
    double wrap_angle(double a) const;
};

// ─────────────────────────────────────────────────────────────────────────────
// Plant  — nonlinear RK4 integrator; exposes PlantState* for sensors
//
// NOTE: Plant must be declared (and thus constructed) before IMUSensor and
//       EncoderSensor in Top so that get_state() is valid when sensors init.
// ─────────────────────────────────────────────────────────────────────────────
SC_MODULE(Plant) {
public:
    sc_core::sc_fifo_in<ControlCommand>    control_in;
    sc_core::sc_fifo_in<DisturbanceTorque> disturbance_in;

    std::uint64_t deadline_misses_ = 0;
    std::uint64_t cmds_consumed_   = 0;

    // Fall detection — set true once |θ| has exceeded fall_threshold_rad
    // continuously for fall_dwell_s. Read by Telemetry::end_of_simulation().
    bool             fell_      = false;
    sc_core::sc_time fall_time_ = sc_core::SC_ZERO_TIME;
    double           theta_max_abs_ = 0.0;

    SC_HAS_PROCESS(Plant);
    Plant(sc_core::sc_module_name name,
          const SimConfig& scfg, const PhysicsConfig& pcfg);
    ~Plant();

    const PlantState* get_state() const { return &state_; }

    void run();

private:
    SimConfig     scfg_;
    PhysicsConfig pcfg_;
    std::mt19937  rng_;
    std::normal_distribution<double> compute_dist_;

    PlantState state_;
    double current_force_ = 0.0;

    // Fall-detection helper: dwell timer integrating contiguous time spent
    // above the fall threshold. Reset to zero whenever |θ| drops back inside.
    sc_core::sc_time fall_dwell_acc_ = sc_core::SC_ZERO_TIME;

    std::ofstream csv_;

    using Vec4 = Eigen::Vector4d;
    Vec4 dynamics(const Vec4& x, double F, double tau_d) const;
    Vec4 rk4_step(const Vec4& x, double F, double tau_d, double dt) const;

    void write_csv_header();
    void write_csv_row(double t);
};

// ─────────────────────────────────────────────────────────────────────────────
// Disturbance  — fires scheduled torque impulses into the disturbance FIFO
// ─────────────────────────────────────────────────────────────────────────────
SC_MODULE(Disturbance) {
public:
    sc_core::sc_fifo_out<DisturbanceTorque> out;

    SC_HAS_PROCESS(Disturbance);
    Disturbance(sc_core::sc_module_name name,
                const SimConfig& scfg, const PhysicsConfig& pcfg);
    void run();

private:
    SimConfig     scfg_;
    PhysicsConfig pcfg_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry  — periodically samples FIFO occupancy; writes a final summary
// (deadline misses, drops, throughput) at end_of_simulation().
// ─────────────────────────────────────────────────────────────────────────────
SC_MODULE(Telemetry) {
public:
    SC_HAS_PROCESS(Telemetry);
    Telemetry(sc_core::sc_module_name name,
              const SimConfig& scfg,
              sc_core::sc_fifo<IMUSample>*         imu_fifo,
              sc_core::sc_fifo<EncoderSample>*     enc_fifo,
              sc_core::sc_fifo<ControlCommand>*    ctrl_fifo,
              sc_core::sc_fifo<DisturbanceTorque>* dist_fifo,
              IMUSensor*     imu_mod,
              EncoderSensor* enc_mod,
              FusionControl* fc_mod,
              Plant*         plant_mod);

    void run();
    void end_of_simulation() override;

private:
    SimConfig scfg_;
    sc_core::sc_fifo<IMUSample>*         imu_fifo_;
    sc_core::sc_fifo<EncoderSample>*     enc_fifo_;
    sc_core::sc_fifo<ControlCommand>*    ctrl_fifo_;
    sc_core::sc_fifo<DisturbanceTorque>* dist_fifo_;
    IMUSensor*     imu_mod_;
    EncoderSensor* enc_mod_;
    FusionControl* fc_mod_;
    Plant*         plant_mod_;
    std::ofstream  fifo_csv_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Top  — owns all FIFOs and modules; wires everything in constructor
//
// Member declaration order matters: Plant must come before IMUSensor/Encoder
// so its constructor (and therefore get_state()) runs first. Telemetry must
// come last so all module pointers are valid when its ctor runs.
// ─────────────────────────────────────────────────────────────────────────────
SC_MODULE(Top) {
public:
    SimConfig     scfg;
    PhysicsConfig pcfg;

    sc_core::sc_fifo<IMUSample>         imu_fifo;
    sc_core::sc_fifo<EncoderSample>     enc_fifo;
    sc_core::sc_fifo<ControlCommand>    control_fifo;
    sc_core::sc_fifo<DisturbanceTorque> disturbance_fifo;

    Plant         plant;
    IMUSensor     imu;
    EncoderSensor encoder;
    FusionControl fusion_ctrl;
    Disturbance   disturbance;
    Telemetry     telemetry;

    Top(sc_core::sc_module_name name,
        const SimConfig& s, const PhysicsConfig& p);
};
