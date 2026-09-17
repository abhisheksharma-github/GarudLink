/**
 * @file ScenarioManager.hpp
 * @brief Thread-safe scenario orchestrator, scheduled threat dispatcher, and runtime injection controller.
 */

#pragma once

#include "Entities.hpp"
#include "ScenarioConfig.hpp"
#include "ShipDataBus.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace SDN::Engine {

/**
 * @struct ManualSpawnRequest
 * @brief Thread-safe queue entry for on-the-fly manual target injections.
 */
struct ManualSpawnRequest {
    std::string id;
    std::string type;
    Math::Vector3D position;
    Math::Vector3D velocity;
    double speedMps{180.0};
    double rcsM2{0.5};
    std::string evasionProfile{"NONE"};
    double evasionFrequencyHz{0.5};
    double evasionAmplitudeG{6.0};
    bool activeJamming{false};
    double jammerPowerW{0.0};
};

/**
 * @class ScenarioManager
 * @brief Coordinates declarative configuration lifecycle and lock-free/thread-safe real-time injections.
 */
class ScenarioManager {
public:
    explicit ScenarioManager(std::shared_ptr<Network::ShipDataBus> dataBus);

    // Scenario Configuration
    bool loadScenario(const Config::ScenarioConfig& config, std::vector<std::shared_ptr<Entities::Entity>>& outEntities, 
                      std::shared_ptr<Entities::RadarSystem>& outRadar);

    bool loadScenarioFromFile(const std::string& filepath, std::vector<std::shared_ptr<Entities::Entity>>& outEntities,
                              std::shared_ptr<Entities::RadarSystem>& outRadar, std::string& outError);

    // Real-Time Update Tick (Processes Scheduled Target Spawns)
    void update(double simTimeSec, std::vector<std::shared_ptr<Entities::Entity>>& activeEntities);

    // Manual Operator Injection (Thread-Safe)
    void queueManualSpawn(const ManualSpawnRequest& request);

    [[nodiscard]] const Config::ScenarioConfig& getCurrentConfig() const noexcept { return currentConfig_; }
    [[nodiscard]] size_t getPendingSpawnCount() const;

private:
    std::shared_ptr<Entities::Entity> instantiateTarget(const Config::TargetConfig& tgt);

    std::shared_ptr<Network::ShipDataBus> dataBus_;
    Config::ScenarioConfig currentConfig_{};

    mutable std::mutex scheduleMutex_;
    std::vector<Config::TargetConfig> pendingScheduledTargets_{};

    mutable std::mutex injectionMutex_;
    std::deque<ManualSpawnRequest> pendingManualInjections_{};
};

} // namespace SDN::Engine
