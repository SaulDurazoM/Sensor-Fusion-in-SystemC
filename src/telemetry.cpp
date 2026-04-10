#include "telemetry.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace sc_core;

namespace {
std::string csv_escape(const std::string& value) {
    bool needs_quotes = value.find(',') != std::string::npos ||
                        value.find('"') != std::string::npos ||
                        value.find('\n') != std::string::npos;
    if (!needs_quotes) {
        return value;
    }

    std::string escaped = "\"";
    for (char c : value) {
        if (c == '"') {
            escaped += "\"\"";
        } else {
            escaped += c;
        }
    }
    escaped += "\"";
    return escaped;
}
}

Telemetry::Telemetry(const Config& cfg)
    : cfg_(cfg) {
}

double Telemetry::to_ms(const sc_time& t) {
    return t.to_seconds() * 1000.0;
}

double Telemetry::median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    if (n % 2 == 1) {
        return values[n / 2];
    }
    return 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

double Telemetry::percentile(std::vector<double> values, double p) {
    if (values.empty()) {
        return 0.0;
    }
    if (values.size() == 1) {
        return values.front();
    }

    std::sort(values.begin(), values.end());

    const double k = (static_cast<double>(values.size()) - 1.0) * (p / 100.0);
    const std::size_t f = static_cast<std::size_t>(std::floor(k));
    const std::size_t c = static_cast<std::size_t>(std::ceil(k));

    if (f == c) {
        return values[f];
    }

    const double d0 = values[f] * (static_cast<double>(c) - k);
    const double d1 = values[c] * (k - static_cast<double>(f));
    return d0 + d1;
}

void Telemetry::log_event(const sc_time& time, const std::string& block, const std::string& event, const std::string& detail) {
    event_rows_.push_back(EventRow{to_ms(time), block, event, detail});
}

void Telemetry::log_fifo_occupancy(const std::string& fifo_name, const sc_time& time, int occupancy) {
    fifo_rows_.push_back(FifoRow{to_ms(time), fifo_name, occupancy});
}

void Telemetry::log_fifo_overflow(const std::string& fifo_name, const sc_time& time, const std::string& policy) {
    fifo_overflows_[fifo_name] += 1;
    log_event(time, fifo_name, "overflow", "policy=" + policy);
}

void Telemetry::log_control_cycle(
    const sc_time& release_time,
    const sc_time& finish_time,
    const sc_time& period,
    bool missed_deadline
) {
    const double release_ms = to_ms(release_time);
    const double finish_ms = to_ms(finish_time);
    const double exec_time_ms = finish_ms - release_ms;
    const double period_ms = to_ms(period);

    control_cycles_ += 1;
    if (missed_deadline) {
        control_deadline_misses_ += 1;
    }

    if (have_last_control_release_) {
        const double actual_period_ms = release_ms - last_control_release_ms_;
        control_jitter_ms_.push_back(std::fabs(actual_period_ms - period_ms));
    }

    last_control_release_ms_ = release_ms;
    have_last_control_release_ = true;

    control_rows_.push_back(ControlRow{
        release_ms,
        finish_ms,
        exec_time_ms,
        missed_deadline ? 1 : 0
    });
}

void Telemetry::log_delay(const ControlCommand& cmd) {
    if (!cmd.valid) {
        return;
    }

    const double control_time_ms = to_ms(cmd.control_time);

    if (cmd.fusion_time != SC_ZERO_TIME) {
        const double fusion_to_command_delay_ms = control_time_ms - to_ms(cmd.fusion_time);
        fusion_to_command_delays_ms_.push_back(fusion_to_command_delay_ms);
    }

    bool have_any_sensor_time = false;
    double newest_sensor_time_ms = 0.0;

    if (cmd.has_imu) {
        newest_sensor_time_ms = to_ms(cmd.imu_time);
        have_any_sensor_time = true;
    }

    if (cmd.has_gps) {
        const double gps_time_ms = to_ms(cmd.gps_time);
        if (!have_any_sensor_time || gps_time_ms > newest_sensor_time_ms) {
            newest_sensor_time_ms = gps_time_ms;
        }
        have_any_sensor_time = true;
    }

    if (have_any_sensor_time) {
        const double sensor_to_command_delay_ms = control_time_ms - newest_sensor_time_ms;
        sensor_to_command_delays_ms_.push_back(sensor_to_command_delay_ms);

        double fusion_to_command_delay_ms = 0.0;
        if (cmd.fusion_time != SC_ZERO_TIME) {
            fusion_to_command_delay_ms = control_time_ms - to_ms(cmd.fusion_time);
        }

        delay_rows_.push_back(DelayRow{
            control_time_ms,
            sensor_to_command_delay_ms,
            fusion_to_command_delay_ms
        });
    }
}

void Telemetry::write_csvs() const {
    std::filesystem::path out_dir = std::filesystem::path(cfg_.output_root) / cfg_.case_name;
    std::filesystem::create_directories(out_dir);

    {
        std::ofstream f(out_dir / "event_log.csv");
        f << "time_ms,block,event,detail\n";
        for (const auto& row : event_rows_) {
            f << std::fixed << std::setprecision(6)
              << row.time_ms << ","
              << csv_escape(row.block) << ","
              << csv_escape(row.event) << ","
              << csv_escape(row.detail) << "\n";
        }
    }

    {
        std::ofstream f(out_dir / "fifo_usage.csv");
        f << "time_ms,fifo,occupancy\n";
        for (const auto& row : fifo_rows_) {
            f << std::fixed << std::setprecision(6)
              << row.time_ms << ","
              << csv_escape(row.fifo) << ","
              << row.occupancy << "\n";
        }
    }

    {
        std::ofstream f(out_dir / "control_log.csv");
        f << "release_ms,finish_ms,exec_time_ms,deadline_missed\n";
        for (const auto& row : control_rows_) {
            f << std::fixed << std::setprecision(6)
              << row.release_ms << ","
              << row.finish_ms << ","
              << row.exec_time_ms << ","
              << row.deadline_missed << "\n";
        }
    }

    {
        std::ofstream f(out_dir / "delay_log.csv");
        f << "control_time_ms,sensor_to_command_delay_ms,fusion_to_command_delay_ms\n";
        for (const auto& row : delay_rows_) {
            f << std::fixed << std::setprecision(6)
              << row.control_time_ms << ","
              << row.sensor_to_command_delay_ms << ","
              << row.fusion_to_command_delay_ms << "\n";
        }
    }

    {
        std::ofstream f(out_dir / "fifo_overflows.csv");
        f << "fifo,overflow_count\n";
        for (const auto& kv : fifo_overflows_) {
            f << csv_escape(kv.first) << "," << kv.second << "\n";
        }
    }

    {
        std::ofstream f(out_dir / "summary.csv");
        f << "metric,value\n";
        f << "control_cycles," << control_cycles_ << "\n";
        f << "control_deadline_misses," << control_deadline_misses_ << "\n";
        f << "sensor_to_command_median_ms," << median(sensor_to_command_delays_ms_) << "\n";
        f << "sensor_to_command_p95_ms," << percentile(sensor_to_command_delays_ms_, 95.0) << "\n";
        f << "sensor_to_command_p99_ms," << percentile(sensor_to_command_delays_ms_, 99.0) << "\n";
        f << "fusion_to_command_median_ms," << median(fusion_to_command_delays_ms_) << "\n";
        f << "fusion_to_command_p95_ms," << percentile(fusion_to_command_delays_ms_, 95.0) << "\n";
        f << "fusion_to_command_p99_ms," << percentile(fusion_to_command_delays_ms_, 99.0) << "\n";
        f << "control_jitter_median_ms," << median(control_jitter_ms_) << "\n";
        f << "control_jitter_p95_ms," << percentile(control_jitter_ms_, 95.0) << "\n";
        f << "control_jitter_p99_ms," << percentile(control_jitter_ms_, 99.0) << "\n";
    }
}

void Telemetry::print_summary() const {
    std::cout << "\n=== CASE: " << cfg_.case_name << " ===\n";
    std::cout << "Control cycles              : " << control_cycles_ << "\n";
    std::cout << "Control deadline misses     : " << control_deadline_misses_ << "\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Sensor->command median (ms) : " << median(sensor_to_command_delays_ms_) << "\n";
    std::cout << "Sensor->command p95 (ms)    : " << percentile(sensor_to_command_delays_ms_, 95.0) << "\n";
    std::cout << "Sensor->command p99 (ms)    : " << percentile(sensor_to_command_delays_ms_, 99.0) << "\n";
    std::cout << "Fusion->command median (ms) : " << median(fusion_to_command_delays_ms_) << "\n";
    std::cout << "Fusion->command p95 (ms)    : " << percentile(fusion_to_command_delays_ms_, 95.0) << "\n";
    std::cout << "Fusion->command p99 (ms)    : " << percentile(fusion_to_command_delays_ms_, 99.0) << "\n";
    std::cout << "Control jitter median (ms)  : " << median(control_jitter_ms_) << "\n";
    std::cout << "Control jitter p95 (ms)     : " << percentile(control_jitter_ms_, 95.0) << "\n";
    std::cout << "Control jitter p99 (ms)     : " << percentile(control_jitter_ms_, 99.0) << "\n";

    std::cout << "FIFO overflows:\n";
    if (fifo_overflows_.empty()) {
        std::cout << "  none\n";
    } else {
        for (const auto& kv : fifo_overflows_) {
            std::cout << "  " << kv.first << ": " << kv.second << "\n";
        }
    }
}