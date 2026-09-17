/**
 * @file ScenarioManager.cpp
 * @brief Implementation of ScenarioManager thread-safe target lifecycle and manual injection handler.
 */

#include "ScenarioManager.hpp"
#include "ThreatLibrary.hpp"

#include <algorithm>
#include <iostream>

namespace SDN::Engine {

ScenarioManager::ScenarioManager(std::shared_ptr<Network::ShipDataBus> dataBus)
    : dataBus_(std::move(dataBus)) {
    if (!dataBus_) {
        dataBus_ = std::make_shared<Network::ShipDataBus>(10000);
    }
}

bool ScenarioManager::loadScenario(
    const Config::ScenarioConfig& config,
    std::vector<std::shared_ptr<Entities::Entity>>& outEntities,
    std::shared_ptr<Entities::RadarSystem>& outRadar
) {
    currentConfig_ = config;

    // 1. Build and configure Primary Naval Radar
    const auto& rdrCfg = config.ownship.radar;
    Entities::RadarOperationalMode rdrMode = Entities::RadarOperationalMode::SURVEILLANCE_360;
    if (rdrCfg.scanMode == "SECTOR_CUE") rdrMode = Entities::RadarOperationalMode::SECTOR_CUE;
    else if (rdrCfg.scanMode == "STT" || rdrCfg.scanMode == "STT_TRACK_LOCK") rdrMode = Entities::RadarOperationalMode::STT_TRACK_LOCK;

    outRadar = std::make_shared<Entities::RadarSystem>(
        rdrCfg.id,
        Entities::Classification::FRIENDLY,
        config.ownship.position,
        rdrCfg.transmitterPowerW,
        rdrCfg.antennaGainDbi,
        rdrCfg.frequencyHz,
        std::pow(10.0, (rdrCfg.noiseFloorDbm - 30.0) / 10.0), // dBm to Watts
        rdrCfg.azimuthBeamwidthDeg,
        rdrCfg.elevationBeamwidthDeg,
        rdrCfg.scanRateDegS
    );
    outRadar->setMode(rdrMode);

    outEntities.clear();
    outEntities.push_back(outRadar);

    // 2. Clear and populate target schedules
    {
        std::lock_guard<std::mutex> lock(scheduleMutex_);
        pendingScheduledTargets_.clear();

        for (const auto& tgtCfg : config.targets) {
            if (tgtCfg.spawnTimeS <= 0.0) {
                // Immediate spawn at T=0
                auto entity = instantiateTarget(tgtCfg);
                if (entity) outEntities.push_back(entity);
            } else {
                // Schedule for runtime dispatch
                pendingScheduledTargets_.push_back(tgtCfg);
            }
        }
    }

    // 3. Announce scenario initialization on data bus
    if (dataBus_) {
        const std::string initSentence = Network::formatSentence("CS", "INIT", {
            config.simulation.name,
            std::to_string(config.targets.size()),
            config.defenseDoctrine.doctrineMode
        });
        dataBus_->publishRaw(initSentence);
    }

    return true;
}

bool ScenarioManager::loadScenarioFromFile(
    const std::string& filepath,
    std::vector<std::shared_ptr<Entities::Entity>>& outEntities,
    std::shared_ptr<Entities::RadarSystem>& outRadar,
    std::string& outError
) {
    auto config = Config::ScenarioConfig::loadFromFile(filepath, outError);
    if (!outError.empty()) {
        return false;
    }
    return loadScenario(config, outEntities, outRadar);
}

std::shared_ptr<Entities::Entity> ScenarioManager::instantiateTarget(const Config::TargetConfig& tgt) {
    Entities::SwerlingModel swerling = Entities::SwerlingModel::SWERLING_1;
    if (tgt.type == "CRUISE_MISSILE" || tgt.type == "ASM") swerling = Entities::SwerlingModel::SWERLING_3;
    else if (tgt.type == "BALLISTIC") swerling = Entities::SwerlingModel::SWERLING_4;

    auto drone = std::make_shared<Entities::Drone>(
        tgt.id,
        Entities::Classification::HOSTILE,
        tgt.initialPosition,
        tgt.speedMps,
        tgt.rcsM2,
        (tgt.type == "CRUISE_MISSILE" ? 650.0 : (tgt.type == "BALLISTIC" ? 1500.0 : 150.0)),
        tgt.type,
        swerling
    );

    drone->setVelocity(tgt.initialVelocity);

    // Apply Evasion Profile
    if (tgt.evasionProfile == "WEAVE_6G" || tgt.evasionProfile == "BARREL_ROLL" || tgt.evasionAmplitudeG > 0.0) {
        drone->setEvasiveMode(true);
        drone->setEvasiveParameters(tgt.evasionFrequencyHz, tgt.evasionAmplitudeG);
    }

    // Apply Active Jamming
    if (tgt.activeJamming) {
        drone->setJammingEnabled(true);
        drone->setJammerPowerWatts(tgt.jammerPowerW);
    }

    // Set heading towards ownship
    drone->addWaypoint({0.0, 0.0, -10.0});

    return drone;
}

void ScenarioManager::queueManualSpawn(const ManualSpawnRequest& request) {
    std::lock_guard<std::mutex> lock(injectionMutex_);
    pendingManualInjections_.push_back(request);
}

void ScenarioManager::update(double simTimeSec, std::vector<std::shared_ptr<Entities::Entity>>& activeEntities) {
    // 1. Process Scheduled Target Spawns
    {
        std::lock_guard<std::mutex> lock(scheduleMutex_);
        for (auto it = pendingScheduledTargets_.begin(); it != pendingScheduledTargets_.end();) {
            if (simTimeSec >= it->spawnTimeS) {
                auto entity = instantiateTarget(*it);
                if (entity) {
                    activeEntities.push_back(entity);

                    if (dataBus_) {
                        const std::string spawnSentence = Network::formatSentence("CS", "SPAWN", {
                            it->id, it->type,
                            std::to_string(static_cast<int>(it->initialPosition.x)),
                            std::to_string(static_cast<int>(it->initialPosition.y)),
                            std::to_string(static_cast<int>(it->initialPosition.z))
                        });
                        dataBus_->publishRaw(spawnSentence);
                    }
                }
                it = pendingScheduledTargets_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // 2. Process Manual Runtime Injections from CLI / Web UI
    {
        std::lock_guard<std::mutex> lock(injectionMutex_);
        while (!pendingManualInjections_.empty()) {
            auto req = pendingManualInjections_.front();
            pendingManualInjections_.pop_front();

            Config::TargetConfig tgt;
            tgt.id = req.id;
            tgt.type = req.type;
            tgt.spawnTimeS = simTimeSec;
            tgt.initialPosition = req.position;
            tgt.initialVelocity = req.velocity;
            tgt.speedMps = req.speedMps;
            tgt.rcsM2 = req.rcsM2;
            tgt.evasionProfile = req.evasionProfile;
            tgt.evasionFrequencyHz = req.evasionFrequencyHz;
            tgt.evasionAmplitudeG = req.evasionAmplitudeG;
            tgt.activeJamming = req.activeJamming;
            tgt.jammerPowerW = req.jammerPowerW;

            auto entity = instantiateTarget(tgt);
            if (entity) {
                activeEntities.push_back(entity);

                if (dataBus_) {
                    const std::string manualSpawn = Network::formatSentence("CS", "MANUAL_SPAWN", {
                        req.id, req.type,
                        std::to_string(static_cast<int>(req.position.x)),
                        std::to_string(static_cast<int>(req.position.y))
                    });
                    dataBus_->publishRaw(manualSpawn);
                }
            }
        }
    }
}

size_t ScenarioManager::getPendingSpawnCount() const {
    std::lock_guard<std::mutex> lock(scheduleMutex_);
    return pendingScheduledTargets_.size();
}

} // namespace SDN::Engine
