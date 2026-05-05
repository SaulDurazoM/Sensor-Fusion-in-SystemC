#pragma once

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
    std::mt19937  rng_{ 42 };
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

    SC_HAS_PROCESS(EncoderSensor);
    EncoderSensor(sc_core::sc_module_name name,
                  const SimConfig& scfg, const PlantState* state);
    void run();

private:
    SimConfig     scfg_;
    const PlantState* state_;
    std::uint64_t seq_ = 0;
    std::mt19937  rng_{ 99 };
    std::normal_distribution<double> compute_dist_;
    std::ofstream csv_;
};

// ─────────────────────────────────────────────────────────────────────────────
// FusionControl  — complementary filter + dual PID, runs at control_period
// ─────────────────────────────────────────────────────────────────────────────
#define USE_FULL_STATE false
SC_MODULE(FusionControl) {
public:
    sc_core::sc_fifo_in<IMUSample>       imu_in;
    sc_core::sc_fifo_in<EncoderSample>   enc_in;
    sc_core::sc_fifo_out<ControlCommand> out;

    SC_HAS_PROCESS(FusionControl);
    FusionControl(sc_core::sc_module_name name,
                  const SimConfig& scfg, const PhysicsConfig& pcfg , const PlantState* state);
    void run();

private:
    SimConfig     scfg_;
    PhysicsConfig pcfg_;
    std::uint64_t seq_ = 0;
    std::mt19937  rng_{ 7 };
    std::normal_distribution<double> compute_normal_dist_;
    std::normal_distribution<double> compute_disturbed_dist_;

    // Latest sensor readings — persist across ticks so filter always has data
    IMUSample     last_imu_;
    EncoderSample last_enc_;
    EncoderSample last_enc_prev_;  // previous encoder sample for z_dot finite difference

    double last_cmd_force_ = 0.0;  // force from previous tick; used for model-based θ̈

    //Alternatively use purely the full state feedback to calculate commands (used for PID constant verification)
    const PlantState* state_; //used when USE_FULL_STATE = true

    // Complementary filter state
    double gyro_ema_      = 0.0;  // EMA-filtered gyro, alpha=0.5 (~3-sample window)
    double theta_est_     = 0.0;
    double theta_from_accel = 0.0; //estimated angle just from accelerometer
    double a_x_corrected    = 0.0;
    double a_y_corrected;
    double theta_dot_est_ = 0.0;
    double theta_ddot_est_ = 0.0;  // model-based (EOM solve); used next tick for accel bias removal
    double z_est_         = 0.0;
    double z_dot_est_     = 0.0;
    double z_ddot_est_    = 0.0;   // stored each tick for next tick's theta bias removal

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

    SC_HAS_PROCESS(Plant);
    Plant(sc_core::sc_module_name name,
          const SimConfig& scfg, const PhysicsConfig& pcfg);
    ~Plant();

    const PlantState* get_state() const { return &state_; }

    void run();

private:
    SimConfig     scfg_;
    PhysicsConfig pcfg_;
    std::mt19937  rng_{ 13 };
    std::normal_distribution<double> compute_dist_;

    PlantState state_;
    double current_force_ = 0.0;

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
// Top  — owns all FIFOs and modules; wires everything in constructor
//
// Member declaration order matters: Plant must come before IMUSensor/Encoder
// so its constructor (and therefore get_state()) runs first.
// ─────────────────────────────────────────────────────────────────────────────
SC_MODULE(Top) {
public:
    SimConfig     scfg;
    PhysicsConfig pcfg;

    sc_core::sc_fifo<IMUSample>         imu_fifo;
    sc_core::sc_fifo<EncoderSample>     enc_fifo;
    sc_core::sc_fifo<ControlCommand>    control_fifo;
    sc_core::sc_fifo<DisturbanceTorque> disturbance_fifo;

    Plant         plant;        // constructed first — sensors need get_state()
    IMUSensor     imu;
    EncoderSensor encoder;
    FusionControl fusion_ctrl;
    Disturbance   disturbance;

    Top(sc_core::sc_module_name name,
        const SimConfig& s, const PhysicsConfig& p);
};
