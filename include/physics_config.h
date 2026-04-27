#pragma once

#include <cmath>
#include <vector>

struct DisturbanceEvent {
    double time_s;
    double torque_nm;
    double duration_s;
};

struct PhysicsConfig {
    // ── Physical parameters ────────────────────────────────────────────────
    double m_c = 1.0;    // cart mass (kg)
    double m_p = 0.3;    // pendulum mass (kg)
    double L   = 0.5;    // pendulum length, pivot to tip (m); CoM at L/2
    double d   = 0.02;   // rod diameter (m)
    double g   = 9.81;   // gravity (m/s²)
    double mu  = 0.05;   // Coulomb friction coefficient

    // ── Derived inertia (computed inline) ────────────────────────────────
    double I_cm()    const { double r = d * 0.5; return (1.0/12.0) * m_p * (3.0*r*r + L*L); }
    double I_pivot() const { return I_cm() + m_p * (L/2.0) * (L/2.0); }

    // ── Initial conditions [θ, θ̇, z, ż] ─────────────────────────────────
    double theta_0     = 0.05;   // small initial tilt (rad)
    double theta_dot_0 = 0.0;
    double z_0         = 0.0;
    double z_dot_0     = 0.0;

    // ── Disturbance schedule ──────────────────────────────────────────────
    // Populated at startup by loading a CSV — see disturbance_csv in SimConfig.
    std::vector<DisturbanceEvent> disturbances;
};
