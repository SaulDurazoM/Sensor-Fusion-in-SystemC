#include <climits>
#include <fstream>
#include <iostream>
#include <limits>
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

static void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [case] [duration_ms] [stress_k] [output_label] [seed]\n"
              << "  case          one of: normal | stress | burst | stress_k\n"
              << "                 (default: normal)\n"
              << "  duration_ms   simulation duration in ms (default: from SimConfig)\n"
              << "  stress_k      compute-scale factor (only used with case=stress_k);\n"
              << "                values >1 push the pipeline past nominal stress\n"
              << "  output_label  override the case_name used as the output subfolder.\n"
              << "                Useful for the stress sweep so each k goes to its own\n"
              << "                results/ subdirectory (e.g. \"stress_k2.5\").\n"
              << "  seed          master RNG seed for all stochastic distributions\n"
              << "                (default: 42). Use different seeds across runs to\n"
              << "                Monte-Carlo over compute-time and noise realizations.\n";
}

int sc_main(int argc, char* argv[]) {
    SimConfig     scfg = SimConfig::normal();
    PhysicsConfig pcfg;
    double        stress_k = 1.0;   // only consulted when case==stress_k
    std::string   case_name_arg;

    if (argc >= 2) {
        case_name_arg = argv[1];
        if (case_name_arg == "-h" || case_name_arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (argc >= 3) {
        try {
            const double dur_ms = std::stod(argv[2]);
            const double max_ms = static_cast<double>(ULLONG_MAX) / 1.0e9;
            if (dur_ms <= max_ms) {
                // Apply duration after the case factory below so it's not overwritten.
            } else {
                std::cerr << "Duration would overflow; ignoring.\n";
            }
        } catch (...) {
            std::cerr << "Invalid duration '" << argv[2] << "'; ignoring.\n";
        }
    }

    if (argc >= 4) {
        try { stress_k = std::stod(argv[3]); }
        catch (...) { std::cerr << "Invalid stress_k '" << argv[3] << "'; using 1.0.\n"; }
    }

    if      (case_name_arg.empty() || case_name_arg == "normal") scfg = SimConfig::normal();
    else if (case_name_arg == "stress")   scfg = SimConfig::stress();
    else if (case_name_arg == "burst")    scfg = SimConfig::burst();
    else if (case_name_arg == "stress_k") scfg = SimConfig::stress_factor(stress_k);
    else {
        std::cerr << "Unknown case '" << case_name_arg << "'.\n";
        print_usage(argv[0]);
        return 1;
    }

    if (argc >= 3) {
        try {
            const double dur_ms = std::stod(argv[2]);
            const double max_ms = static_cast<double>(ULLONG_MAX) / 1.0e9;
            if (dur_ms <= max_ms)
                scfg.simulation_duration = sc_time(dur_ms, SC_MS);
        } catch (...) { /* already warned above */ }
    }

    if (argc >= 5) {
        scfg.case_name = argv[4];   // override output subfolder name
    }

    if (argc >= 6) {
        try {
            const long s = std::stol(argv[5]);
            if (s < 0 || s > static_cast<long>(std::numeric_limits<unsigned int>::max())) {
                std::cerr << "Seed " << s << " out of range; using default 42.\n";
            } else {
                scfg.seed = static_cast<unsigned int>(s);
            }
        } catch (...) {
            std::cerr << "Invalid seed '" << argv[5] << "'; using default 42.\n";
        }
    }

    load_disturbances(pcfg, scfg.disturbance_csv);

    std::cout << "Case:     " << scfg.case_name           << "\n";
    std::cout << "Duration: " << scfg.simulation_duration << "\n";
    std::cout << "Seed:     " << scfg.seed                << "\n";
    if (case_name_arg == "stress_k")
        std::cout << "stress_k: " << stress_k << "\n";

    Top top("top", scfg, pcfg);
    sc_start(scfg.simulation_duration);

    // Trigger end_of_simulation() callbacks (e.g. Telemetry::end_of_simulation()
    // writes summary.csv). Without this, sc_start() returns due to time-out
    // and the kernel does not invoke end_of_simulation() in all SystemC
    // implementations.
    sc_stop();

    std::cout << "Simulation finished at " << sc_time_stamp() << "\n";
    return 0;
}
