#include <iostream>
#include <string>
#include <systemc>

#include "top.h"

using namespace sc_core;

static Config build_config_from_args(int argc, char* argv[]) {
    Config cfg = Config::normal();

    if (argc >= 2) {
        const std::string case_name = argv[1];
        if (case_name == "normal") {
            cfg = Config::normal();
        } else if (case_name == "stress") {
            cfg = Config::stress();
        } else if (case_name == "burst") {
            cfg = Config::burst();
        } else {
            std::cerr << "Unknown case '" << case_name << "'. Using normal.\n";
            cfg = Config::normal();
        }
    }

    if (argc >= 3) {
        const double duration_ms = std::stod(argv[2]);
        cfg.simulation_duration = sc_time(duration_ms, SC_MS);
    }

    return cfg;
}

int sc_main(int argc, char* argv[]) {
    Config cfg = build_config_from_args(argc, argv);

    std::cout << "Starting SystemC sensor-fusion pipeline\n";
    std::cout << "Case: " << cfg.case_name << "\n";
    std::cout << "Simulation duration: " << cfg.simulation_duration << "\n";

    Top top("top", cfg);

    sc_start(cfg.simulation_duration);

    top.telemetry.write_csvs();
    top.telemetry.print_summary();

    std::cout << "Simulation finished at " << sc_time_stamp() << "\n";
    return 0;
}