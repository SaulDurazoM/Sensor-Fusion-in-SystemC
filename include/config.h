#pragma once

#include <string>
#include <systemc>

enum class OverflowPolicy {
    DropNewest
};

inline std::string to_string(OverflowPolicy policy) {
    switch (policy) {
        case OverflowPolicy::DropNewest:
            return "drop_newest";
        default:
            return "unknown";
    }
}

struct Config {
    std::string case_name = "normal";
    std::string output_root = "results";

    sc_core::sc_time simulation_duration = sc_core::sc_time(10, sc_core::SC_SEC);

    sc_core::sc_time imu_period = sc_core::sc_time(1, sc_core::SC_MS);
    sc_core::sc_time gps_period = sc_core::sc_time(10, sc_core::SC_MS);
    sc_core::sc_time fusion_period = sc_core::sc_time(1, sc_core::SC_MS);
    sc_core::sc_time control_period = sc_core::sc_time(1, sc_core::SC_MS);

    int imu_fifo_depth = 32;
    int gps_fifo_depth = 32;
    int fusion_fifo_depth = 32;
    int control_fifo_depth = 32;

    OverflowPolicy overflow_policy = OverflowPolicy::DropNewest;

    sc_core::sc_time fusion_compute = sc_core::sc_time(80, sc_core::SC_US);
    sc_core::sc_time control_compute = sc_core::sc_time(100, sc_core::SC_US);
    sc_core::sc_time plant_compute = sc_core::sc_time(20, sc_core::SC_US);

    bool stress_enabled = false;
    sc_core::sc_time stress_fusion_extra = sc_core::sc_time(0, sc_core::SC_US);
    sc_core::sc_time stress_control_extra = sc_core::sc_time(0, sc_core::SC_US);

    bool burst_enabled = false;
    sc_core::sc_time burst_start = sc_core::sc_time(2, sc_core::SC_SEC);
    sc_core::sc_time burst_end = sc_core::sc_time(3500, sc_core::SC_MS);
    sc_core::sc_time imu_burst_period = sc_core::sc_time(150, sc_core::SC_US);
    sc_core::sc_time gps_burst_period = sc_core::sc_time(2, sc_core::SC_MS);

    static Config normal() {
        Config cfg;
        cfg.case_name = "normal";
        return cfg;
    }

    static Config stress() {
        Config cfg;
        cfg.case_name = "stress";
        cfg.stress_enabled = true;
        cfg.stress_fusion_extra = sc_core::sc_time(550, sc_core::SC_US);
        cfg.stress_control_extra = sc_core::sc_time(700, sc_core::SC_US);
        return cfg;
    }

    static Config burst() {
        Config cfg;
        cfg.case_name = "burst";
        cfg.burst_enabled = true;
        cfg.imu_burst_period = sc_core::sc_time(150, sc_core::SC_US);
        cfg.gps_burst_period = sc_core::sc_time(2, sc_core::SC_MS);
        return cfg;
    }
};