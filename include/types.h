#pragma once

#include <cstdint>
#include <ostream>
#include <systemc>

struct IMUSample {
    std::uint64_t seq = 0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
    sc_core::sc_time timestamp = sc_core::SC_ZERO_TIME;
    bool valid = false;
    bool burst = false;
};

struct GPSSample {
    std::uint64_t seq = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double velocity = 0.0;
    sc_core::sc_time timestamp = sc_core::SC_ZERO_TIME;
    bool valid = false;
    bool burst = false;
};

struct FusionState {
    std::uint64_t seq = 0;
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    sc_core::sc_time imu_time = sc_core::SC_ZERO_TIME;
    sc_core::sc_time gps_time = sc_core::SC_ZERO_TIME;
    sc_core::sc_time fusion_time = sc_core::SC_ZERO_TIME;
    bool has_imu = false;
    bool has_gps = false;
    bool valid = false;
};

struct ControlCommand {
    std::uint64_t seq = 0;
    double throttle = 0.0;
    double steering = 0.0;
    sc_core::sc_time imu_time = sc_core::SC_ZERO_TIME;
    sc_core::sc_time gps_time = sc_core::SC_ZERO_TIME;
    sc_core::sc_time fusion_time = sc_core::SC_ZERO_TIME;
    sc_core::sc_time control_time = sc_core::SC_ZERO_TIME;
    bool has_imu = false;
    bool has_gps = false;
    bool valid = false;
};

inline std::ostream& operator<<(std::ostream& os, const IMUSample& s) {
    os << "IMUSample(seq=" << s.seq
       << ", ax=" << s.ax
       << ", ay=" << s.ay
       << ", az=" << s.az
       << ", gx=" << s.gx
       << ", gy=" << s.gy
       << ", gz=" << s.gz
       << ", t=" << s.timestamp
       << ", burst=" << s.burst
       << ", valid=" << s.valid << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const GPSSample& s) {
    os << "GPSSample(seq=" << s.seq
       << ", lat=" << s.latitude
       << ", lon=" << s.longitude
       << ", alt=" << s.altitude
       << ", vel=" << s.velocity
       << ", t=" << s.timestamp
       << ", burst=" << s.burst
       << ", valid=" << s.valid << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const FusionState& s) {
    os << "FusionState(seq=" << s.seq
       << ", x=" << s.x
       << ", y=" << s.y
       << ", heading=" << s.heading
       << ", speed=" << s.speed
       << ", fusion_t=" << s.fusion_time
       << ", valid=" << s.valid << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const ControlCommand& c) {
    os << "ControlCommand(seq=" << c.seq
       << ", throttle=" << c.throttle
       << ", steering=" << c.steering
       << ", control_t=" << c.control_time
       << ", valid=" << c.valid << ")";
    return os;
}