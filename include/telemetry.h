#pragma once

#include <map>
#include <string>
#include <vector>
#include <systemc>

#include "config.h"
#include "types.h"

struct EventRow {
    double time_ms = 0.0;
    std::string block;
    std::string event;
    std::string detail;
};

struct FifoRow {
    double time_ms = 0.0;
    std::string fifo;
    int occupancy = 0;
};

struct ControlRow {
    double release_ms = 0.0;
    double finish_ms = 0.0;
    double exec_time_ms = 0.0;
    int deadline_missed = 0;
};

struct DelayRow {
    double control_time_ms = 0.0;
    double sensor_to_command_delay_ms = 0.0;
    double fusion_to_command_delay_ms = 0.0;
};

class Telemetry {
public:
    explicit Telemetry(const Config& cfg);

    void log_event(const sc_core::sc_time& time, const std::string& block, const std::string& event, const std::string& detail = "");
    void log_fifo_occupancy(const std::string& fifo_name, const sc_core::sc_time& time, int occupancy);
    void log_fifo_overflow(const std::string& fifo_name, const sc_core::sc_time& time, const std::string& policy);

    void log_control_cycle(
        const sc_core::sc_time& release_time,
        const sc_core::sc_time& finish_time,
        const sc_core::sc_time& period,
        bool missed_deadline
    );

    void log_delay(const ControlCommand& cmd);

    void write_csvs() const;
    void print_summary() const;

private:
    Config cfg_;

    std::vector<double> sensor_to_command_delays_ms_;
    std::vector<double> fusion_to_command_delays_ms_;
    std::vector<double> control_jitter_ms_;

    std::size_t control_cycles_ = 0;
    std::size_t control_deadline_misses_ = 0;

    bool have_last_control_release_ = false;
    double last_control_release_ms_ = 0.0;

    std::map<std::string, int> fifo_overflows_;
    std::vector<EventRow> event_rows_;
    std::vector<FifoRow> fifo_rows_;
    std::vector<ControlRow> control_rows_;
    std::vector<DelayRow> delay_rows_;

    static double to_ms(const sc_core::sc_time& t);
    static double median(std::vector<double> values);
    static double percentile(std::vector<double> values, double p);
};