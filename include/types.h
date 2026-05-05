#pragma once

#include <cstdint>
#include <ostream>
#include <systemc>

// ── Shared plant state (read by sensors via const pointer) ─────────────────
// Lives in Plant; no FIFO needed — SystemC is cooperatively scheduled so
// there is no concurrent access between wait() points.
struct PlantState {
    double theta      = 0.0;   // pendulum angle from vertical (rad, CCW+)
    double theta_dot  = 0.0;   // angular rate (rad/s)
    double z          = 0.0;   // cart position along rail (m, rightward+)
    double z_dot      = 0.0;   // cart velocity (m/s)
    double z_ddot     = 0.0;   // cart acceleration — read by IMU to sim accel_rail
    double theta_ddot = 0.0;   // angular acceleration — read by IMU for gravity projection
    double tau_disturbance = 0.0;
    bool   disturbed  = false;
};

// ── IMU sample (1 kHz, noisy) ──────────────────────────────────────────────
struct IMUSample {
    std::uint64_t seq  = 0;
    double omega       = 0.0;  // gyro: θ̇ + noise (rad/s)
    double a_x_prime   = 0.0;  // accelerometer perpendicular to rod (x̂'=(cosθ,sinθ), +x at upright):    −L·θ̈ + g·sin θ + z̈·cos θ + noise
    double a_y_prime   = 0.0;  // accelerometer along rod (ŷ'=(−sinθ,cosθ), +y at upright):              −L·θ̇² + g·cos θ − z̈·sin θ + noise
    bool   disturbed   = false; // propagated from PlantState; drives FC compute distribution
    sc_core::sc_time timestamp = sc_core::SC_ZERO_TIME;
    bool valid = false;
};

// ── Encoder sample (100 Hz, quantised) ────────────────────────────────────
struct EncoderSample {
    std::uint64_t seq    = 0;
    double z_quantized   = 0.0;   // z rounded to nearest encoder_resolution (m)
    sc_core::sc_time timestamp = sc_core::SC_ZERO_TIME;
    bool valid = false;
};

// ── Control command (force on cart in Newtons) ────────────────────────────
struct ControlCommand {
    std::uint64_t seq = 0;
    double force      = 0.0;
    sc_core::sc_time timestamp = sc_core::SC_ZERO_TIME;
    bool valid = false;
};

// ── Disturbance torque (scheduled impulses at pivot) ─────────────────────
struct DisturbanceTorque {
    double tau = 0.0;   // τ_d (N·m)
    sc_core::sc_time timestamp = sc_core::SC_ZERO_TIME;
};

// ── Stream operators (required by sc_fifo even when unused in traces) ─────
inline std::ostream& operator<<(std::ostream& os, const IMUSample& s) {
    return os << "IMUSample(seq=" << s.seq << ",omega=" << s.omega
              << ",ax'=" << s.a_x_prime << ",ay'=" << s.a_y_prime << ")";
}
inline std::ostream& operator<<(std::ostream& os, const EncoderSample& s) {
    return os << "EncoderSample(seq=" << s.seq << ",z=" << s.z_quantized << ")";
}
inline std::ostream& operator<<(std::ostream& os, const ControlCommand& c) {
    return os << "ControlCommand(seq=" << c.seq << ",F=" << c.force << ")";
}
inline std::ostream& operator<<(std::ostream& os, const DisturbanceTorque& d) {
    return os << "DisturbanceTorque(tau=" << d.tau << ")";
}
