#include <climits>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <systemc>

#include "physics_config.h"
#include "sim_config.h"
#include "top.h"

using namespace sc_core;

static void load_disturbances(PhysicsConfig& pcfg, const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Warning: could not open disturbance CSV '" << path << "'; no disturbances loaded.\n";
        return;
    }
    std::string line;
    std::getline(f, line);  // skip header row
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tok;
        try {
            DisturbanceEvent ev{};
            std::getline(ss, tok, ','); ev.time_s     = std::stod(tok);
            std::getline(ss, tok, ','); ev.torque_nm  = std::stod(tok);
            std::getline(ss, tok, ','); ev.duration_s = std::stod(tok);
            pcfg.disturbances.push_back(ev);
        } catch (...) {
            std::cerr << "Warning: skipping malformed disturbance row: " << line << "\n";
        }
    }
    std::cout << "Loaded " << pcfg.disturbances.size()
              << " disturbance events from '" << path << "'\n";
}

int sc_main(int argc, char* argv[]) {
    SimConfig     scfg = SimConfig::normal();
    PhysicsConfig pcfg;

    if (argc >= 2) {
        const std::string name = argv[1];
        if (name == "normal") {
            scfg = SimConfig::normal();
        } else {
            std::cerr << "Unknown case '" << name << "'. Using normal.\n";
        }
    }

    if (argc >= 3) {
        try {
            const double dur_ms = std::stod(argv[2]);
            const double max_ms = static_cast<double>(ULLONG_MAX) / 1.0e9;
            if (dur_ms <= max_ms)
                scfg.simulation_duration = sc_time(dur_ms, SC_MS);
            else
                std::cerr << "Duration would overflow; ignoring.\n";
        } catch (...) {
            std::cerr << "Invalid duration '" << argv[2] << "'; ignoring.\n";
        }
    }

    load_disturbances(pcfg, scfg.disturbance_csv);

    std::cout << "Case:     " << scfg.case_name          << "\n";
    std::cout << "Duration: " << scfg.simulation_duration << "\n";

    Top top("top", scfg, pcfg);
    sc_start(scfg.simulation_duration);

    std::cout << "Simulation finished at " << sc_time_stamp() << "\n";
    return 0;
}
