#pragma once

#include <systemc>

#include "config.h"
#include "telemetry.h"
#include "types.h"

SC_MODULE(IMUSource) {
public:
    sc_core::sc_fifo_out<IMUSample> out;

    SC_HAS_PROCESS(IMUSource);

    IMUSource(sc_core::sc_module_name name, const Config& cfg, Telemetry& telemetry);

    void run();

private:
    Config cfg_;
    Telemetry& telemetry_;
    std::uint64_t seq_ = 0;

    bool in_burst_window(const sc_core::sc_time& now) const;
};

SC_MODULE(GPSSource) {
public:
    sc_core::sc_fifo_out<GPSSample> out;

    SC_HAS_PROCESS(GPSSource);

    GPSSource(sc_core::sc_module_name name, const Config& cfg, Telemetry& telemetry);

    void run();

private:
    Config cfg_;
    Telemetry& telemetry_;
    std::uint64_t seq_ = 0;

    bool in_burst_window(const sc_core::sc_time& now) const;
};

SC_MODULE(Fusion) {
public:
    sc_core::sc_fifo_in<IMUSample> imu_in;
    sc_core::sc_fifo_in<GPSSample> gps_in;
    sc_core::sc_fifo_out<FusionState> out;

    SC_HAS_PROCESS(Fusion);

    Fusion(sc_core::sc_module_name name, const Config& cfg, Telemetry& telemetry);

    void run();

private:
    Config cfg_;
    Telemetry& telemetry_;

    std::uint64_t seq_ = 0;
    IMUSample last_imu_;
    GPSSample last_gps_;
    bool have_imu_ = false;
    bool have_gps_ = false;
};

SC_MODULE(Control) {
public:
    sc_core::sc_fifo_in<FusionState> in;
    sc_core::sc_fifo_out<ControlCommand> out;

    SC_HAS_PROCESS(Control);

    Control(sc_core::sc_module_name name, const Config& cfg, Telemetry& telemetry);

    void run();

private:
    Config cfg_;
    Telemetry& telemetry_;
    std::uint64_t seq_ = 0;
};

SC_MODULE(Plant) {
public:
    sc_core::sc_fifo_in<ControlCommand> in;

    SC_HAS_PROCESS(Plant);

    Plant(sc_core::sc_module_name name, const Config& cfg, Telemetry& telemetry);

    void run();

private:
    Config cfg_;
    Telemetry& telemetry_;

    double x_ = 0.0;
    double y_ = 0.0;
    double heading_ = 0.0;
};

SC_MODULE(Top) {
public:
    Config cfg;
    Telemetry telemetry;

    sc_core::sc_fifo<IMUSample> imu_fifo;
    sc_core::sc_fifo<GPSSample> gps_fifo;
    sc_core::sc_fifo<FusionState> fusion_fifo;
    sc_core::sc_fifo<ControlCommand> control_fifo;

    IMUSource imu;
    GPSSource gps;
    Fusion fusion;
    Control control;
    Plant plant;

    Top(sc_core::sc_module_name name, const Config& config);
};