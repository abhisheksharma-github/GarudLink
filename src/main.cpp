/**
 * @file main.cpp
 * @brief Real-time interactive main simulation driver for Project GarudLink SDN & CMS.
 *
 * Supports:
 *  - Declarative configuration file loading via CLI (`--config <path>` / `-c <path>`)
 *  - Automatic graceful fallback to default scenario or built-in test parameters
 *  - Real-time interactive CLI shell (`load`, `spawn`, `radar`, `fire`, `jam`, `status`, `help`, `quit`)
 *  - High-frequency 100 Hz simulation engine loop with bidirectional WebSocket bridge
 */

#include "Entities.hpp"
#include "Math3D.hpp"
#include "ScenarioConfig.hpp"
#include "ScenarioManager.hpp"
#include "ShipDataBus.hpp"
#include "SimulationEngine.hpp"
#include "ThreatLibrary.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {
    std::atomic<bool> g_shutdownRequested{false};

    void signalHandler(int signum) {
        if (signum == SIGINT || signum == SIGTERM) {
            std::cout << "\n[MAIN] Shutdown signal received. Gracefully terminating GarudLink...\n" << std::flush;
            g_shutdownRequested.store(true);
        }
    }

    void printHelp() {
        std::cout << "\n================== GARUDLINK INTERACTIVE CLI COMMANDS ==================\n"
                  << "  load <file.json>                       : Hot-reload scenario configuration file\n"
                  << "  spawn <type> <x> <y> <z> <vx> <vy> <vz> <rcs> : Inject threat into tactical grid\n"
                  << "         Types: UAV | CRUISE_MISSILE | BALLISTIC | DRONE_SWARM\n"
                  << "         Coordinates in meters (NED), velocities in m/s, RCS in m^2\n"
                  << "  radar <mode> [center_deg] [width_deg]  : Dynamic Radar Tasking\n"
                  << "         Modes: SURVEILLANCE_360 | SECTOR_CUE | STT\n"
                  << "  fire <target_id>                       : Authorize interceptor release on target\n"
                  << "  jam <target_id> <power_w>              : Simulate active EW/RF jamming on target\n"
                  << "  evade <target_id> [g_load] [freq_hz]   : Enable evasive weaving maneuvers\n"
                  << "  status                                 : Display active tracks, radar & interceptor status\n"
                  << "  help                                   : Show this commands list\n"
                  << "  quit / exit                            : Terminate simulation session\n"
                  << "========================================================================\n\n";
    }

    std::vector<std::string> tokenize(const std::string& line) {
        std::vector<std::string> tokens;
        std::istringstream iss(line);
        std::string tok;
        while (iss >> tok) {
            tokens.push_back(tok);
        }
        return tokens;
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    int wsPort = 8080;
    bool jsonStreamMode = false;
    std::string configPath = "";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--json" || arg == "-j") {
            jsonStreamMode = true;
        } else if ((arg == "--port" || arg == "-p") && i + 1 < argc) {
            wsPort = std::stoi(argv[++i]);
        } else if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: tactical_sim [options]\n"
                      << "Options:\n"
                      << "  --config, -c <path>   Load declarative scenario JSON config\n"
                      << "  --port, -p <port>     WebSocket server port (default: 8080)\n"
                      << "  --json, -j            Stream JSON telemetry output to stdout\n"
                      << "  --help, -h            Show help banner\n";
            return 0;
        }
    }

    if (!jsonStreamMode) {
        std::cout << "=====================================================================\n"
                  << "   PROJECT GARUDLINK: REAL-TIME TACTICAL SHIP DATA NETWORK (SDN)    \n"
                  << "   Multi-Domain C++20 GNC Engine, Phased Array Radar & CMS Bridge   \n"
                  << "=====================================================================\n\n";
    }

    // 1. Initialize Ship Data Bus
    auto dataBus = std::make_shared<SDN::Network::ShipDataBus>(10000);

    // 2. Initialize Simulation Engine
    auto engine = std::make_unique<SDN::Engine::SimulationEngine>(dataBus);

    // 3. Scenario Ingestion / Default Fallback
    bool scenarioLoaded = false;
    if (!configPath.empty()) {
        std::string err;
        if (engine->loadScenarioFile(configPath, err)) {
            if (!jsonStreamMode) {
                std::cout << "[CONFIG] Loaded declarative scenario file: " << configPath << "\n";
            }
            scenarioLoaded = true;
        } else {
            std::cerr << "[CONFIG ERROR] Failed to load '" << configPath << "': " << err << "\n"
                      << "[CONFIG] Falling back to default scenario...\n";
        }
    }

    if (!scenarioLoaded) {
        // Attempt loading default scenario file if present
        std::string defaultPath = "scenarios/default_scenario.json";
        std::string err;
        if (engine->loadScenarioFile(defaultPath, err)) {
            if (!jsonStreamMode) {
                std::cout << "[CONFIG] Auto-loaded reference scenario: " << defaultPath << "\n";
            }
            scenarioLoaded = true;
        } else {
            // Graceful internal fallback setup
            if (!jsonStreamMode) {
                std::cout << "[CONFIG] Initializing built-in default naval battle group baseline...\n";
            }
            auto fallbackConfig = SDN::Config::ScenarioConfig::createDefault();
            engine->loadScenarioConfig(fallbackConfig);
            scenarioLoaded = true;
        }
    }

    // 4. Subscribe to CMS Engagement Events for Terminal Logging
    dataBus->subscribe(SDN::Network::Talkers::COMBAT_SYSTEM, [&](const SDN::Network::NetworkMessage& msg) {
        if (!jsonStreamMode) {
            if (msg.sentenceType == "ENGAGE" && msg.fields.size() >= 3) {
                std::cout << "\n[CMS ALERT] AUTO-ENGAGEMENT TRIGGERED! Target " << msg.fields[1]
                          << " breached " << msg.fields[2] << "m perimeter. Launched Interceptor "
                          << msg.fields[0] << ".\n> " << std::flush;
            } else if (msg.sentenceType == "KILL" && msg.fields.size() >= 2) {
                std::cout << "\n[CMS SUCCESS] TARGET NEUTRALIZED! Interceptor " << msg.fields[0]
                          << " destroyed threat " << msg.fields[1] << ".\n> " << std::flush;
            } else if (msg.sentenceType == "MISS" && msg.fields.size() >= 3) {
                std::cout << "\n[CMS NOTICE] INTERCEPTOR MISSED Target " << msg.fields[1]
                          << " by " << msg.fields[2] << " meters.\n> " << std::flush;
            } else if (msg.sentenceType == "SPAWN" && msg.fields.size() >= 2) {
                std::cout << "\n[CMS RADAR] NEW THREAT INJECTED: " << msg.fields[0]
                          << " (" << msg.fields[1] << ")\n> " << std::flush;
            }
        }
    });

    if (jsonStreamMode) {
        engine->setJsonTelemetryCallback([](const std::string& json) {
            std::cout << json << "\n" << std::flush;
        });
    }

    if (!jsonStreamMode) {
        std::cout << "[SYSTEM] Interactive GNC & Radar Engine Active.\n"
                  << "  - Bidirectional WebSocket Server : ws://localhost:" << wsPort << "\n"
                  << "  - Combat Management Doctrine     : " << (engine->isAutoEngagementEnabled() ? "AUTO_DEFENSE" : "MANUAL_AUTH") << "\n"
                  << "  - Engagement Radius Threshold    : " << engine->getEngagementRangeThreshold() / 1000.0 << " km\n"
                  << "  - AAR Data Flight Recorder       : Active (writes to engagement_report.csv)\n\n"
                  << "Type 'help' for command list. Press Ctrl+C or type 'quit' to exit.\n\n";
    }

    // 5. Launch Engine with WebSocket Server
    engine->startAsync(wsPort);

    // 6. Interactive CLI Input Thread (when not in raw JSON stream mode)
    std::thread cliThread;
    if (!jsonStreamMode) {
        cliThread = std::thread([&]() {
            std::string line;
            while (!g_shutdownRequested.load()) {
                std::cout << "> " << std::flush;
                if (!std::getline(std::cin, line)) {
                    break;
                }
                if (g_shutdownRequested.load()) break;

                auto tokens = tokenize(line);
                if (tokens.empty()) continue;

                std::string cmd = tokens[0];
                for (auto& c : cmd) c = static_cast<char>(std::tolower(c));

                if (cmd == "quit" || cmd == "exit") {
                    std::cout << "[MAIN] Exiting on operator command...\n";
                    g_shutdownRequested.store(true);
                    break;
                } else if (cmd == "help" || cmd == "?") {
                    printHelp();
                } else if (cmd == "load") {
                    if (tokens.size() < 2) {
                        std::cout << "[CLI ERROR] Usage: load <scenario_path.json>\n";
                    } else {
                        std::string err;
                        if (engine->loadScenarioFile(tokens[1], err)) {
                            std::cout << "[CLI SUCCESS] Loaded scenario: " << tokens[1] << "\n";
                        } else {
                            std::cout << "[CLI ERROR] Failed to load scenario: " << err << "\n";
                        }
                    }
                } else if (cmd == "spawn") {
                    if (tokens.size() < 9) {
                        std::cout << "[CLI ERROR] Usage: spawn <type> <x> <y> <z> <vx> <vy> <vz> <rcs>\n"
                                  << "     Types: UAV, CRUISE_MISSILE, BALLISTIC, DRONE_SWARM\n";
                    } else {
                        try {
                            SDN::Engine::ManualSpawnRequest req;
                            req.typeStr = tokens[1];
                            req.position = {std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4])};
                            req.velocity = {std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7])};
                            req.rcs = std::stod(tokens[8]);
                            req.customId = "";
                            engine->injectManualTarget(req);
                            std::cout << "[CLI SUCCESS] Spawn request injected for type: " << req.typeStr << "\n";
                        } catch (const std::exception& e) {
                            std::cout << "[CLI ERROR] Invalid numeric parameters: " << e.what() << "\n";
                        }
                    }
                } else if (cmd == "radar") {
                    if (tokens.size() < 2) {
                        std::cout << "[CLI ERROR] Usage: radar <mode> [center_deg] [width_deg]\n"
                                  << "     Modes: SURVEILLANCE_360 | SECTOR_CUE | STT\n";
                    } else {
                        std::string modeStr = tokens[1];
                        double center = (tokens.size() >= 3) ? std::stod(tokens[2]) : 0.0;
                        double width = (tokens.size() >= 4) ? std::stod(tokens[3]) : 45.0;

                        if (modeStr == "SURVEILLANCE_360" || modeStr == "360" || modeStr == "SURV") {
                            engine->setRadarMode(SDN::Entities::RadarOperationalMode::SURVEILLANCE_360);
                            std::cout << "[CLI RADAR] Mode updated to SURVEILLANCE_360 (Continuous 360 deg scan)\n";
                        } else if (modeStr == "SECTOR_CUE" || modeStr == "SECTOR") {
                            engine->setRadarMode(SDN::Entities::RadarOperationalMode::SECTOR_CUE, center, width);
                            std::cout << "[CLI RADAR] Mode updated to SECTOR_CUE (Center: " << center 
                                      << " deg, Width: " << width << " deg)\n";
                        } else if (modeStr == "STT" || modeStr == "TRACK") {
                            std::string tgtId = (tokens.size() >= 3) ? tokens[2] : "";
                            engine->setRadarMode(SDN::Entities::RadarOperationalMode::SINGLE_TARGET_TRACK, 0.0, 5.0, tgtId);
                            std::cout << "[CLI RADAR] Mode updated to SINGLE_TARGET_TRACK (Target: " << tgtId << ")\n";
                        } else {
                            std::cout << "[CLI ERROR] Unknown radar mode '" << modeStr << "'\n";
                        }
                    }
                } else if (cmd == "fire") {
                    if (tokens.size() < 2) {
                        std::cout << "[CLI ERROR] Usage: fire <target_id>\n";
                    } else {
                        engine->authorizeEngagement(tokens[1]);
                        std::cout << "[CLI FIRE] Manual interceptor authorization dispatched for: " << tokens[1] << "\n";
                    }
                } else if (cmd == "jam") {
                    if (tokens.size() < 3) {
                        std::cout << "[CLI ERROR] Usage: jam <target_id> <power_watts>\n";
                    } else {
                        try {
                            double power = std::stod(tokens[2]);
                            engine->setTargetJamming(tokens[1], power > 0.0, power);
                            std::cout << "[CLI EW] Target " << tokens[1] << " jamming set to " << power << " Watts\n";
                        } catch (const std::exception& e) {
                            std::cout << "[CLI ERROR] Invalid power value: " << e.what() << "\n";
                        }
                    }
                } else if (cmd == "evade") {
                    if (tokens.size() < 2) {
                        std::cout << "[CLI ERROR] Usage: evade <target_id> [g_load=6.0] [freq_hz=0.5]\n";
                    } else {
                        double gLoad = (tokens.size() >= 3) ? std::stod(tokens[2]) : 6.0;
                        double freq = (tokens.size() >= 4) ? std::stod(tokens[3]) : 0.5;
                        engine->setTargetEvasion(tokens[1], true, gLoad, freq);
                        std::cout << "[CLI TACTIC] Target " << tokens[1] << " evasive weave activated (" 
                                  << gLoad << "G @ " << freq << " Hz)\n";
                    }
                } else if (cmd == "status") {
                    double t = engine->getSimulationTimeSec();
                    auto entities = engine->getEntities();
                    std::cout << "\n================ TACTICAL STATUS REPORT (T=" 
                              << std::fixed << std::setprecision(1) << t << "s) ================\n";
                    std::cout << "Auto-Engagement: " << (engine->isAutoEngagementEnabled() ? "ENABLED" : "MANUAL")
                              << " | Perimeter: " << engine->getEngagementRangeThreshold() / 1000.0 << " km\n";
                    std::cout << "Active Air/Surface Entities:\n";
                    for (const auto& entity : entities) {
                        if (entity && entity->isActive()) {
                            auto pos = entity->getPosition();
                            auto vel = entity->getVelocity();
                            std::cout << "  - [" << entity->getId() << "] Type: " << entity->getTypeString()
                                      << " | Pos: (" << std::setprecision(0) << pos.x << ", " << pos.y << ", " << pos.z << ") m"
                                      << " | Range: " << std::setprecision(2) << pos.magnitude() / 1000.0 << " km"
                                      << " | Vel: " << std::setprecision(1) << vel.magnitude() << " m/s"
                                      << " | RCS: " << entity->getRcs() << " m^2\n";
                        }
                    }
                    std::cout << "=================================================================\n\n";
                } else {
                    std::cout << "[CLI ERROR] Unknown command '" << cmd << "'. Type 'help' for available commands.\n";
                }
            }
        });
    }

    // 7. Main loop heartbeat
    while (!g_shutdownRequested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 8. Graceful Teardown
    if (cliThread.joinable()) {
        cliThread.detach(); // Allow clean termination without waiting for blocking getline
    }

    engine->stop();
    engine->exportAfterActionReportCsv("engagement_report.csv");

    if (!jsonStreamMode) {
        std::cout << "\n[MAIN] Simulation halted. Engagement report exported to 'engagement_report.csv'.\n";
    }

    return 0;
}
