#include "top.h"

#include <cmath>
#include <iostream>
#include <sstream>

using namespace sc_core;

namespace {
double to_ms(const sc_time& t) {
    return t.to_seconds() * 1000.0;
}

template <typename T>
int fifo_occupancy_after_write(const sc_fifo_out<T>& port, int depth) {
    return depth - port.num_free();
}
}

IMUSource::IMUSource(sc_module_name name, const Config& cfg, Telemetry& telemetry)
    : sc_module(name), cfg_(cfg), telemetry_(telemetry) {
    SC_THREAD(run);
}

bool IMUSource::in_burst_window(const sc_time& now) const {
    return cfg_.burst_enabled && now >= cfg_.burst_start && now < cfg_.burst_end;
}

void IMUSource::run() {
    while (true) {
        const bool burst = in_burst_window(sc_time_stamp());
        const sc_time period = burst ? cfg_.imu_burst_period : cfg_.imu_period;
        wait(period);

        IMUSample sample;
        sample.seq = seq_++;
        sample.ax = 0.1 * static_cast<double>(sample.seq);
        sample.ay = 0.2 * static_cast<double>(sample.seq);
        sample.az = 9.81;
        sample.gx = 0.01;
        sample.gy = 0.02;
        sample.gz = 0.03 + 0.0001 * static_cast<double>(sample.seq);
        sample.timestamp = sc_time_stamp();
        sample.valid = true;
        sample.burst = burst;

        if (!out.nb_write(sample)) {
            telemetry_.log_fifo_overflow("imu_fifo", sc_time_stamp(), to_string(cfg_.overflow_policy));
            telemetry_.log_event(sc_time_stamp(), "imu", "drop", "seq=" + std::to_string(sample.seq));
        } else {
            telemetry_.log_fifo_occupancy("imu_fifo", sc_time_stamp(), fifo_occupancy_after_write(out, cfg_.imu_fifo_depth));
            telemetry_.log_event(sc_time_stamp(), "imu", "produce", "seq=" + std::to_string(sample.seq) + ", burst=" + std::to_string(static_cast<int>(burst)));
        }
    }
}

GPSSource::GPSSource(sc_module_name name, const Config& cfg, Telemetry& telemetry)
    : sc_module(name), cfg_(cfg), telemetry_(telemetry) {
    SC_THREAD(run);
}

bool GPSSource::in_burst_window(const sc_time& now) const {
    return cfg_.burst_enabled && now >= cfg_.burst_start && now < cfg_.burst_end;
}

void GPSSource::run() {
    while (true) {
        const bool burst = in_burst_window(sc_time_stamp());
        const sc_time period = burst ? cfg_.gps_burst_period : cfg_.gps_period;
        wait(period);

        GPSSample sample;
        sample.seq = seq_++;
        sample.latitude = 32.2319 + 0.00001 * static_cast<double>(sample.seq);
        sample.longitude = -110.9501 + 0.00001 * static_cast<double>(sample.seq);
        sample.altitude = 728.0;
        sample.velocity = 5.0 + 0.001 * static_cast<double>(sample.seq);
        sample.timestamp = sc_time_stamp();
        sample.valid = true;
        sample.burst = burst;

        if (!out.nb_write(sample)) {
            telemetry_.log_fifo_overflow("gps_fifo", sc_time_stamp(), to_string(cfg_.overflow_policy));
            telemetry_.log_event(sc_time_stamp(), "gps", "drop", "seq=" + std::to_string(sample.seq));
        } else {
            telemetry_.log_fifo_occupancy("gps_fifo", sc_time_stamp(), fifo_occupancy_after_write(out, cfg_.gps_fifo_depth));
            telemetry_.log_event(sc_time_stamp(), "gps", "produce", "seq=" + std::to_string(sample.seq) + ", burst=" + std::to_string(static_cast<int>(burst)));
        }
    }
}

Fusion::Fusion(sc_module_name name, const Config& cfg, Telemetry& telemetry)
    : sc_module(name), cfg_(cfg), telemetry_(telemetry) {
    SC_THREAD(run);
}

void Fusion::run() {
    sc_time next_release = SC_ZERO_TIME;

    while (true) {
        if (sc_time_stamp() < next_release) {
            wait(next_release - sc_time_stamp());
        }
        const sc_time release_time = sc_time_stamp();
        next_release += cfg_.fusion_period;

        IMUSample imu_sample;
        while (imu_in.nb_read(imu_sample)) {
            last_imu_ = imu_sample;
            have_imu_ = true;
            telemetry_.log_fifo_occupancy("imu_fifo", sc_time_stamp(), imu_in.num_available());
        }

        GPSSample gps_sample;
        while (gps_in.nb_read(gps_sample)) {
            last_gps_ = gps_sample;
            have_gps_ = true;
            telemetry_.log_fifo_occupancy("gps_fifo", sc_time_stamp(), gps_in.num_available());
        }

        sc_time compute_time = cfg_.fusion_compute;
        if (cfg_.stress_enabled) {
            compute_time += cfg_.stress_fusion_extra;
        }
        if (compute_time > SC_ZERO_TIME) {
            wait(compute_time);
        }

        FusionState state;
        state.seq = seq_++;
        state.valid = have_imu_ || have_gps_;
        state.fusion_time = sc_time_stamp();

        if (have_gps_) {
            state.x = last_gps_.latitude;
            state.y = last_gps_.longitude;
            state.speed = last_gps_.velocity;
            state.gps_time = last_gps_.timestamp;
            state.has_gps = true;
        }

        if (have_imu_) {
            state.heading = last_imu_.gz * 10.0;
            state.imu_time = last_imu_.timestamp;
            state.has_imu = true;
        }

        telemetry_.log_event(release_time, "fusion", "start", "");

        if (!out.nb_write(state)) {
            telemetry_.log_fifo_overflow("fusion_fifo", sc_time_stamp(), to_string(cfg_.overflow_policy));
            telemetry_.log_event(sc_time_stamp(), "fusion", "drop", "seq=" + std::to_string(state.seq));
        } else {
            telemetry_.log_fifo_occupancy("fusion_fifo", sc_time_stamp(), fifo_occupancy_after_write(out, cfg_.fusion_fifo_depth));

            std::ostringstream oss;
            oss << "seq=" << state.seq
                << ", valid=" << static_cast<int>(state.valid)
                << ", release_ms=" << to_ms(release_time);
            telemetry_.log_event(sc_time_stamp(), "fusion", "finish", oss.str());
        }
    }
}

Control::Control(sc_module_name name, const Config& cfg, Telemetry& telemetry)
    : sc_module(name), cfg_(cfg), telemetry_(telemetry) {
    SC_THREAD(run);
}

void Control::run() {
    sc_time next_release = SC_ZERO_TIME;

    while (true) {
        if (sc_time_stamp() < next_release) {
            wait(next_release - sc_time_stamp());
        }
        const sc_time release_time = sc_time_stamp();
        next_release += cfg_.control_period;

        FusionState latest_state;
        bool have_state = false;
        FusionState temp;

        while (in.nb_read(temp)) {
            latest_state = temp;
            have_state = true;
            telemetry_.log_fifo_occupancy("fusion_fifo", sc_time_stamp(), in.num_available());
        }

        sc_time compute_time = cfg_.control_compute;
        if (cfg_.stress_enabled) {
            compute_time += cfg_.stress_control_extra;
        }
        if (compute_time > SC_ZERO_TIME) {
            wait(compute_time);
        }

        ControlCommand cmd;
        cmd.seq = seq_++;
        cmd.control_time = sc_time_stamp();
        cmd.valid = have_state && latest_state.valid;

        if (cmd.valid) {
            cmd.throttle = 1.0;
            cmd.steering = -0.10 * latest_state.heading;
            cmd.imu_time = latest_state.imu_time;
            cmd.gps_time = latest_state.gps_time;
            cmd.fusion_time = latest_state.fusion_time;
            cmd.has_imu = latest_state.has_imu;
            cmd.has_gps = latest_state.has_gps;
        }

        if (!out.nb_write(cmd)) {
            telemetry_.log_fifo_overflow("control_fifo", sc_time_stamp(), to_string(cfg_.overflow_policy));
            telemetry_.log_event(sc_time_stamp(), "control", "drop", "seq=" + std::to_string(cmd.seq));
        } else {
            telemetry_.log_fifo_occupancy("control_fifo", sc_time_stamp(), fifo_occupancy_after_write(out, cfg_.control_fifo_depth));
        }

        const sc_time finish_time = sc_time_stamp();
        const bool missed_deadline = finish_time > (release_time + cfg_.control_period);

        telemetry_.log_control_cycle(release_time, finish_time, cfg_.control_period, missed_deadline);
        telemetry_.log_delay(cmd);

        std::ostringstream oss;
        oss << "seq=" << cmd.seq
            << ", valid=" << static_cast<int>(cmd.valid)
            << ", missed=" << static_cast<int>(missed_deadline);
        telemetry_.log_event(sc_time_stamp(), "control", "deadlines", oss.str());
    }
}

Plant::Plant(sc_module_name name, const Config& cfg, Telemetry& telemetry)
    : sc_module(name), cfg_(cfg), telemetry_(telemetry) {
    SC_THREAD(run);
}

void Plant::run() {
    while (true) {
        ControlCommand cmd = in.read();
        telemetry_.log_fifo_occupancy("control_fifo", sc_time_stamp(), in.num_available());

        if (cfg_.plant_compute > SC_ZERO_TIME) {
            wait(cfg_.plant_compute);
        }

        if (cmd.valid) {
            x_ += cmd.throttle * 0.01;
            y_ += cmd.throttle * 0.005;
            heading_ += cmd.steering * 0.01;
        }

        std::ostringstream oss;
        oss << "x=" << x_
            << ", y=" << y_
            << ", heading=" << heading_
            << ", valid=" << static_cast<int>(cmd.valid);
        telemetry_.log_event(sc_time_stamp(), "plant", "response_trace", oss.str());
    }
}

Top::Top(sc_module_name name, const Config& config)
    : sc_module(name),
      cfg(config),
      telemetry(cfg),
      imu_fifo(cfg.imu_fifo_depth),
      gps_fifo(cfg.gps_fifo_depth),
      fusion_fifo(cfg.fusion_fifo_depth),
      control_fifo(cfg.control_fifo_depth),
      imu("imu_source", cfg, telemetry),
      gps("gps_source", cfg, telemetry),
      fusion("fusion_block", cfg, telemetry),
      control("control_block", cfg, telemetry),
      plant("plant_block", cfg, telemetry) {
    imu.out(imu_fifo);
    gps.out(gps_fifo);

    fusion.imu_in(imu_fifo);
    fusion.gps_in(gps_fifo);
    fusion.out(fusion_fifo);

    control.in(fusion_fifo);
    control.out(control_fifo);

    plant.in(control_fifo);

    telemetry.log_fifo_occupancy("imu_fifo", SC_ZERO_TIME, 0);
    telemetry.log_fifo_occupancy("gps_fifo", SC_ZERO_TIME, 0);
    telemetry.log_fifo_occupancy("fusion_fifo", SC_ZERO_TIME, 0);
    telemetry.log_fifo_occupancy("control_fifo", SC_ZERO_TIME, 0);
}