/**
 * @file SimulationEngine.cpp
 * @brief Production Implementation of Tactical Simulation Engine, CMS Coordinator, and WebSocket Bridge.
 */

#include "SimulationEngine.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace SDN::Engine {

SimulationEngine::SimulationEngine(std::shared_ptr<Network::ShipDataBus> dataBus)
    : dataBus_(std::move(dataBus)) {
    if (!dataBus_) {
        dataBus_ = std::make_shared<Network::ShipDataBus>(10000);
    }
    scenarioManager_ = std::make_shared<ScenarioManager>(dataBus_);
    initBusSubscriptions();
}

SimulationEngine::~SimulationEngine() {
    stop();
}

void SimulationEngine::initBusSubscriptions() {
    if (!dataBus_) return;

    // 1. Ring buffer for recent NMEA bus logs
    dataBus_->subscribe(Network::Talkers::WILDCARD_ALL, [this](const Network::NetworkMessage& msg) {
        std::lock_guard<std::mutex> lock(logMutex_);
        if (recentNmeaLogs_.size() >= MAX_LOG_ENTRIES) {
            recentNmeaLogs_.pop_front();
        }
        recentNmeaLogs_.push_back(msg.toFormattedSentence());
    });

    // 2. CMS subscribes to Radar tracks ($RDR)
    dataBus_->subscribe(Network::Talkers::RADAR, [this](const Network::NetworkMessage& msg) {
        if (msg.sentenceType == "TRK") {
            onRadarTrackMessage(msg);
        }
    });

    // 3. Auto-save AAR report upon engagement resolution
    dataBus_->subscribe(Network::Talkers::COMBAT_SYSTEM, [this](const Network::NetworkMessage& msg) {
        if (msg.sentenceType == "KILL" || msg.sentenceType == "MISS") {
            exportAfterActionReportCsv("engagement_report.csv");
        }
    });
}

// ============================================================================
// Declarative Scenario Configuration Loading
// ============================================================================

bool SimulationEngine::loadScenarioConfig(const Config::ScenarioConfig& config) {
    std::lock_guard<std::mutex> lock(entitiesMutex_);
    std::lock_guard<std::mutex> cmsLock(cmsMutex_);

    activeMissiles_.clear();
    trackTable_.clear();

    bool success = scenarioManager_->loadScenario(config, entities_, primaryRadar_);
    if (success) {
        remainingInterceptors_ = config.defenseDoctrine.interceptorCount;
        autoEngagementEnabled_ = (config.defenseDoctrine.doctrineMode == "AUTO_DEFENSE");
        engagementRangeThresholdM_ = config.defenseDoctrine.maxEngagementRangeM;
    }
    return success;
}

bool SimulationEngine::loadScenarioFile(const std::string& filepath, std::string& outError) {
    auto cfg = Config::ScenarioConfig::loadFromFile(filepath, outError);
    if (!outError.empty()) return false;
    return loadScenarioConfig(cfg);
}

bool SimulationEngine::loadScenarioJson(const std::string& json, std::string& outError) {
    auto cfg = Config::ScenarioConfig::loadFromJsonString(json, outError);
    if (!outError.empty()) return false;
    return loadScenarioConfig(cfg);
}

void SimulationEngine::injectManualTarget(const ManualSpawnRequest& req) {
    scenarioManager_->queueManualSpawn(req);
}

void SimulationEngine::addEntity(std::shared_ptr<Entities::Entity> entity) {
    if (!entity) return;
    std::lock_guard<std::mutex> lock(entitiesMutex_);
    entities_.push_back(entity);

    if (auto missile = std::dynamic_pointer_cast<Entities::GuidedMissile>(entity)) {
        activeMissiles_.push_back(missile);
    }
}

void SimulationEngine::setPrimaryRadar(std::shared_ptr<Entities::RadarSystem> radar) {
    primaryRadar_ = radar;
    if (radar) {
        addEntity(radar);
    }
}

std::vector<std::shared_ptr<Entities::Entity>> SimulationEngine::getEntities() const {
    std::lock_guard<std::mutex> lock(entitiesMutex_);
    return entities_;
}

std::shared_ptr<Entities::Entity> SimulationEngine::findEntity(const std::string& id) const {
    std::lock_guard<std::mutex> lock(entitiesMutex_);
    for (const auto& e : entities_) {
        if (e && e->getId() == id) {
            return e;
        }
    }
    return nullptr;
}

// ============================================================================
// Operator Tasking & Command Handling
// ============================================================================

void SimulationEngine::handleRawJsonCommand(const std::string& json) {
    auto cmd = Protocol::JsonHelper::parseCommand(json);
    handleCommand(cmd);
}

void SimulationEngine::handleCommand(const Protocol::OperatorCommand& cmd) {
    switch (cmd.type) {
        case Protocol::CommandType::LOAD_SCENARIO: {
            std::string err;
            if (!cmd.scenarioJsonContent.empty()) {
                loadScenarioJson(cmd.scenarioJsonContent, err);
            } else if (!cmd.scenarioPath.empty()) {
                loadScenarioFile(cmd.scenarioPath, err);
            }
            if (!err.empty()) {
                std::cerr << "[SimulationEngine] Error loading scenario: " << err << "\n";
            }
            break;
        }

        case Protocol::CommandType::SPAWN_TARGET: {
            ManualSpawnRequest req;
            req.id = cmd.customId.empty() ? ("TGT-" + std::to_string(threatCounter_++)) : cmd.customId;
            req.type = cmd.customType.empty() ? "UAV" : cmd.customType;
            req.position = cmd.spawnPosition;
            req.velocity = cmd.initialVelocity;
            req.speedMps = cmd.speedMps;
            req.rcsM2 = cmd.rcsM2;
            req.evasionProfile = cmd.evasionEnabled ? "WEAVE_6G" : "NONE";
            req.evasionAmplitudeG = cmd.evasionAmplitudeG;
            req.evasionFrequencyHz = cmd.evasionFrequencyHz;
            req.activeJamming = cmd.jammingEnabled;
            req.jammerPowerW = cmd.jammerPowerWatts;
            injectManualTarget(req);
            break;
        }

        case Protocol::CommandType::AUTHORIZE_INTERCEPT:
            authorizeEngagement(cmd.targetId);
            break;

        case Protocol::CommandType::SET_RADAR_MODE:
            setRadarMode(static_cast<Entities::RadarOperationalMode>(cmd.radarMode),
                         cmd.cueAzimuthDeg, cmd.sectorWidthDeg, cmd.targetId);
            break;

        case Protocol::CommandType::TRIGGER_EVASION:
            setTargetEvasion(cmd.targetId, cmd.evasionEnabled, cmd.evasionAmplitudeG, cmd.evasionFrequencyHz);
            break;

        case Protocol::CommandType::DEPLOY_JAMMING:
            setTargetJamming(cmd.targetId, cmd.jammingEnabled, cmd.jammerPowerWatts);
            break;

        case Protocol::CommandType::SET_AUTO_ENGAGE:
            setAutoEngagement(cmd.autoEngage);
            break;

        case Protocol::CommandType::SET_RANGE_THRESHOLD:
            setEngagementRangeThreshold(cmd.rangeThresholdM);
            break;

        case Protocol::CommandType::RESET_SCENARIO: {
            auto defCfg = Config::ScenarioConfig::createDefault();
            loadScenarioConfig(defCfg);
            break;
        }

        default:
            break;
    }
}

std::shared_ptr<Entities::Drone> SimulationEngine::spawnThreat(
    Protocol::ThreatPreset preset, const Math::Vector3D& spawnPos, const std::string& customId
) {
    std::shared_ptr<Entities::Drone> threat;
    std::string id = customId.empty() ? ("TGT-" + std::to_string(threatCounter_++)) : customId;

    switch (preset) {
        case Protocol::ThreatPreset::RECON_QUAD_UAV:
            threat = Threats::ThreatLibrary::createReconQuadUav(id, spawnPos);
            break;

        case Protocol::ThreatPreset::KAMIKAZE_SWARM:
            threat = Threats::ThreatLibrary::createKamikazeDrone(id, spawnPos);
            break;

        case Protocol::ThreatPreset::SEA_SKIMMER_ASM:
            threat = Threats::ThreatLibrary::createSeaSkimmerAsm(id, spawnPos);
            break;

        case Protocol::ThreatPreset::CUSTOM:
        default:
            threat = std::make_shared<Entities::Drone>(
                id, Entities::Classification::HOSTILE, spawnPos, 180.0, 0.5, 300.0, "CUSTOM_THREAT"
            );
            threat->addWaypoint({0.0, 0.0, -20.0});
            break;
    }

    addEntity(threat);

    const std::string spawnSentence = Network::formatSentence("CS", "SPAWN", {
        threat->getId(), threat->getThreatType(),
        std::to_string(static_cast<int>(spawnPos.x)),
        std::to_string(static_cast<int>(spawnPos.y)),
        std::to_string(static_cast<int>(spawnPos.z))
    });
    dataBus_->publishRaw(spawnSentence);

    return threat;
}

void SimulationEngine::setRadarMode(Entities::RadarOperationalMode mode, double cueAzimuth, 
                                     double sectorWidth, const std::string& targetId) {
    if (!primaryRadar_) return;

    switch (mode) {
        case Entities::RadarOperationalMode::SURVEILLANCE_360:
            primaryRadar_->setMode(Entities::RadarOperationalMode::SURVEILLANCE_360);
            break;

        case Entities::RadarOperationalMode::SECTOR_CUE:
            primaryRadar_->setSectorCue(cueAzimuth, sectorWidth);
            break;

        case Entities::RadarOperationalMode::STT_TRACK_LOCK:
            primaryRadar_->setSttLock(targetId);
            break;
    }
}

void SimulationEngine::setTargetEvasion(const std::string& targetId, bool enable, double amplitudeG, double freqHz) {
    auto entity = findEntity(targetId);
    if (auto drone = std::dynamic_pointer_cast<Entities::Drone>(entity)) {
        drone->setEvasiveMode(enable);
        drone->setEvasiveParameters(freqHz, amplitudeG);
    }
}

void SimulationEngine::setTargetJamming(const std::string& targetId, bool enable, double jamPowerWatts) {
    auto entity = findEntity(targetId);
    if (auto drone = std::dynamic_pointer_cast<Entities::Drone>(entity)) {
        drone->setJammingEnabled(enable);
        drone->setJammerPowerWatts(jamPowerWatts);
    }
}

// ============================================================================
// Fire Control / CMS Coordination
// ============================================================================

void SimulationEngine::authorizeEngagement(const std::string& targetId) {
    std::lock_guard<std::mutex> lock(cmsMutex_);
    auto it = trackTable_.find(targetId);
    if (it != trackTable_.end()) {
        launchMissileInterceptor(it->second);
        it->second.engagementAssigned = true;
    } else {
        auto entity = findEntity(targetId);
        if (entity && entity->isActive()) {
            TrackState track;
            track.trackId = targetId;
            const Math::Vector3D relPos = entity->getPosition();
            track.rangeM = relPos.magnitude();
            track.azimuthDeg = std::atan2(relPos.y, relPos.x) * (180.0 / 3.141592653589793);
            if (track.azimuthDeg < 0.0) track.azimuthDeg += 360.0;
            track.elevationDeg = 0.0;
            track.estimatedPos = entity->getPosition();
            track.estimatedVel = entity->getVelocity();
            launchMissileInterceptor(track);
            trackTable_[targetId] = track;
            trackTable_[targetId].engagementAssigned = true;
        }
    }
}

void SimulationEngine::onRadarTrackMessage(const Network::NetworkMessage& msg) {
    if (msg.fields.size() < 4) return;

    try {
        const std::string& targetId = msg.fields[0];
        const double azDeg = std::stod(msg.fields[1]);
        const double elDeg = std::stod(msg.fields[2]);
        const double rangeM = std::stod(msg.fields[3]);
        const double snrDb = (msg.fields.size() > 4) ? std::stod(msg.fields[4]) : 10.0;

        std::lock_guard<std::mutex> lock(cmsMutex_);
        auto& track = trackTable_[targetId];
        track.trackId = targetId;
        track.azimuthDeg = azDeg;
        track.elevationDeg = elDeg;
        track.rangeM = rangeM;
        track.snrDb = snrDb;
        track.lastUpdateSec = simTimeSec_.load();

        // Automated engagement evaluation
        if (autoEngagementEnabled_ && !track.engagementAssigned && rangeM <= engagementRangeThresholdM_ && remainingInterceptors_ > 0) {
            launchMissileInterceptor(track);
            track.engagementAssigned = true;
        }
    } catch (...) {}
}

void SimulationEngine::launchMissileInterceptor(const TrackState& track) {
    if (remainingInterceptors_ <= 0) {
        std::cout << "[CMS WARNING] VLS EXHAUSTED: No interceptors remaining in magazine!\n";
        return;
    }
    remainingInterceptors_--;

    const std::string missileId = "MSL-" + std::to_string(missileLaunchCounter_++);
    const Math::Vector3D launchPos{0.0, 0.0, -12.0};

    const double azRad = track.azimuthDeg * (3.141592653589793 / 180.0);
    const double elRad = track.elevationDeg * (3.141592653589793 / 180.0);
    const double cosEl = std::cos(elRad);

    const Math::Vector3D launchDir{
        std::cos(azRad) * cosEl,
        std::sin(azRad) * cosEl,
        -std::sin(elRad)
    };

    const Math::Vector3D initialVel = launchDir.normalized() * 350.0;

    auto missile = std::make_shared<Entities::GuidedMissile>(
        missileId,
        Entities::Classification::FRIENDLY,
        launchPos,
        initialVel,
        track.trackId,
        3.0,     // 3.0s Booster stage
        4.0,     // 4.0s Sustainer stage
        24000.0, // 24 kN Booster Thrust
        6000.0,  // 6 kN Sustainer Thrust
        40.0,    // 40 kg dry mass
        45.0,    // 45 kg propellant
        4.0,     // N = 4.0 Proportional Nav Ratio
        35.0,    // 35G structural limit
        12.0     // 12m Proximity fuse
    );

    {
        std::lock_guard<std::mutex> lock(entitiesMutex_);
        entities_.push_back(missile);
        activeMissiles_.push_back(missile);
    }

    const std::string engageSentence = Network::formatSentence(
        "CS", "ENGAGE", {missileId, track.trackId, std::to_string(static_cast<int>(track.rangeM))}
    );
    dataBus_->publishRaw(engageSentence);
}

// ============================================================================
// Multi-Rate Physics, Sensor & Telemetry Ticks
// ============================================================================

void SimulationEngine::physicsTick(double dt) {
    std::lock_guard<std::mutex> lock(entitiesMutex_);

    // 1. Process scheduled & manual injections from ScenarioManager
    if (scenarioManager_) {
        scenarioManager_->update(simTimeSec_.load(), entities_);
    }

    // 2. Midcourse target updates for active missiles
    for (auto& missile : activeMissiles_) {
        if (!missile || !missile->isActive()) continue;

        const std::string& tgtId = missile->getTargetId();
        for (const auto& entity : entities_) {
            if (entity && entity->getId() == tgtId && entity->isActive()) {
                missile->setTargetKinematics(entity->getPosition(), entity->getVelocity());
                break;
            }
        }
    }

    // 3. Step physics forward
    for (auto& entity : entities_) {
        if (entity && entity->isActive()) {
            entity->update(dt);
        }
    }

    // 4. Detonation & Intercept Resolution
    for (auto& missile : activeMissiles_) {
        if (missile && missile->hasDetonated()) {
            const std::string& tgtId = missile->getTargetId();
            if (missile->isTargetIntercepted()) {
                for (auto& entity : entities_) {
                    if (entity && entity->getId() == tgtId) {
                        entity->setActive(false);
                        const std::string killMsg = Network::formatSentence("CS", "KILL", {missile->getId(), tgtId});
                        dataBus_->publishRaw(killMsg);
                        break;
                    }
                }
            } else {
                const std::string missMsg = Network::formatSentence("CS", "MISS", {
                    missile->getId(), tgtId, std::to_string(static_cast<int>(missile->getMissDistanceM()))
                });
                dataBus_->publishRaw(missMsg);
            }
        }
    }
}

void SimulationEngine::radarTick(double dt) {
    if (!primaryRadar_ || !primaryRadar_->isActive()) return;

    primaryRadar_->update(dt);

    std::vector<std::shared_ptr<Entities::Entity>> targetsSnapshot;
    {
        std::lock_guard<std::mutex> lock(entitiesMutex_);
        targetsSnapshot = entities_;
    }

    std::vector<std::string> tracks = primaryRadar_->scanTargets(targetsSnapshot);
    for (const auto& trackSentence : tracks) {
        dataBus_->publishRaw(trackSentence);
    }
}

void SimulationEngine::busTelemetryTick() {
    std::vector<std::shared_ptr<Entities::Entity>> entitiesSnapshot;
    {
        std::lock_guard<std::mutex> lock(entitiesMutex_);
        entitiesSnapshot = entities_;
    }

    for (const auto& entity : entitiesSnapshot) {
        if (entity && entity->isActive()) {
            dataBus_->publishRaw(entity->serializeTelemetry());
        }
    }
}

void SimulationEngine::telemetryJsonTick() {
    std::string jsonStr = serializeStateJson();

    if (wsBridge_ && wsBridge_->isRunning()) {
        wsBridge_->broadcastText(jsonStr);
    }

    std::lock_guard<std::mutex> lock(callbackMutex_);
    if (jsonCallback_) {
        jsonCallback_(jsonStr);
    }
}

std::string SimulationEngine::serializeStateJson() const {
    std::ostringstream json;
    json << "{\n";
    json << "  \"sim_time_sec\": " << std::fixed << std::setprecision(2) << simTimeSec_.load() << ",\n";
    json << "  \"scenario_name\": \"" << scenarioManager_->getCurrentConfig().simulation.name << "\",\n";

    // Radar status
    json << "  \"radar\": {\n";
    double radarAz = primaryRadar_ ? primaryRadar_->getCurrentAzimuthDeg() : 0.0;
    std::string modeStr = "SURVEILLANCE_360";
    double cueAz = 0.0, secWidth = 45.0;
    std::string lockedId = "";

    if (primaryRadar_) {
        modeStr = (primaryRadar_->getMode() == Entities::RadarOperationalMode::SURVEILLANCE_360 ? "SURVEILLANCE_360" :
                  (primaryRadar_->getMode() == Entities::RadarOperationalMode::SECTOR_CUE ? "SECTOR_CUE" : "STT_TRACK_LOCK"));
        cueAz = primaryRadar_->getCueAzimuthDeg();
        secWidth = primaryRadar_->getSectorWidthDeg();
        lockedId = primaryRadar_->getLockedTargetId();
    }

    json << "    \"azimuth_deg\": " << std::fixed << std::setprecision(1) << radarAz << ",\n";
    json << "    \"mode\": \"" << modeStr << "\",\n";
    json << "    \"cue_azimuth_deg\": " << std::fixed << std::setprecision(1) << cueAz << ",\n";
    json << "    \"sector_width_deg\": " << std::fixed << std::setprecision(1) << secWidth << ",\n";
    json << "    \"locked_target_id\": \"" << lockedId << "\",\n";
    json << "    \"range_rings_km\": [5, 10, 15, 20, 25]\n";
    json << "  },\n";

    // CMS Fire Control Status
    json << "  \"cms\": {\n";
    json << "    \"auto_engage\": " << (autoEngagementEnabled_ ? "true" : "false") << ",\n";
    json << "    \"threshold_m\": " << std::fixed << std::setprecision(0) << engagementRangeThresholdM_ << ",\n";
    json << "    \"remaining_interceptors\": " << remainingInterceptors_ << "\n";
    json << "  },\n";

    // Entities List
    json << "  \"entities\": [\n";
    {
        std::lock_guard<std::mutex> lock(entitiesMutex_);
        for (size_t i = 0; i < entities_.size(); ++i) {
            const auto& e = entities_[i];
            if (!e) continue;

            std::string type = "GENERIC";
            std::string threatType = "NONE";
            bool evasive = false, jamming = false;
            double rcs = 0.5;

            std::string motorStage = "NONE";
            double lookAngle = 0.0, closingVel = 0.0, rangeToGo = 0.0;
            bool targetLock = false;

            if (auto drone = dynamic_cast<Entities::Drone*>(e.get())) {
                type = "DRONE";
                threatType = drone->getThreatType();
                evasive = drone->isEvasiveModeEnabled();
                jamming = drone->isJammingEnabled();
                rcs = drone->getRcs();
            } else if (dynamic_cast<Entities::RadarSystem*>(e.get())) {
                type = "RADAR";
            } else if (auto msl = dynamic_cast<Entities::GuidedMissile*>(e.get())) {
                type = "MISSILE";
                motorStage = Entities::motorPhaseToString(msl->getMotorPhase());
                lookAngle = msl->getSeekerLookAngleDeg();
                closingVel = msl->getClosingVelocity();
                rangeToGo = msl->getRangeToGo();
                targetLock = msl->hasTargetLock();
            }

            const auto& pos = e->getPosition();
            const auto& vel = e->getVelocity();

            json << "    {\n";
            json << "      \"id\": \"" << e->getId() << "\",\n";
            json << "      \"type\": \"" << type << "\",\n";
            json << "      \"threat_type\": \"" << threatType << "\",\n";
            json << "      \"classification\": \"" << Entities::classificationToString(e->getClassification()) << "\",\n";
            json << "      \"active\": " << (e->isActive() ? "true" : "false") << ",\n";
            json << "      \"position\": [" << std::fixed << std::setprecision(1) << pos.x << ", " << pos.y << ", " << pos.z << "],\n";
            json << "      \"velocity\": [" << std::fixed << std::setprecision(1) << vel.x << ", " << vel.y << ", " << vel.z << "],\n";
            json << "      \"speed_mps\": " << std::fixed << std::setprecision(1) << e->getSpeed() << ",\n";
            json << "      \"heading_deg\": " << std::fixed << std::setprecision(1) << e->getHeadingDeg() << ",\n";
            json << "      \"altitude_m\": " << std::fixed << std::setprecision(1) << e->getAltitudeM() << ",\n";
            json << "      \"rcs_m2\": " << std::fixed << std::setprecision(3) << rcs << ",\n";
            json << "      \"evasive\": " << (evasive ? "true" : "false") << ",\n";
            json << "      \"jamming\": " << (jamming ? "true" : "false") << ",\n";
            json << "      \"motor_stage\": \"" << motorStage << "\",\n";
            json << "      \"look_angle_deg\": " << std::fixed << std::setprecision(1) << lookAngle << ",\n";
            json << "      \"closing_velocity_mps\": " << std::fixed << std::setprecision(1) << closingVel << ",\n";
            json << "      \"range_to_go_m\": " << std::fixed << std::setprecision(1) << rangeToGo << ",\n";
            json << "      \"target_lock\": " << (targetLock ? "true" : "false") << "\n";
            json << "    }" << (i + 1 < entities_.size() ? "," : "") << "\n";
        }
    }
    json << "  ],\n";

    // Recent NMEA Bus Logs
    json << "  \"recent_nmea_logs\": [\n";
    {
        std::lock_guard<std::mutex> lock(logMutex_);
        for (size_t i = 0; i < recentNmeaLogs_.size(); ++i) {
            std::string line = recentNmeaLogs_[i];
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
                line.pop_back();
            }
            json << "    \"" << line << "\"" << (i + 1 < recentNmeaLogs_.size() ? "," : "") << "\n";
        }
    }
    json << "  ]\n";
    json << "}\n";

    return json.str();
}

// ============================================================================
// After-Action Review (AAR) Flight Data Recorder Export
// ============================================================================

bool SimulationEngine::exportAfterActionReportCsv(const std::string& filepath) const {
    std::ofstream csv(filepath);
    if (!csv.is_open()) return false;

    csv << "Timestamp_s,Target_ID,Interceptor_ID,Slant_Range_m,Closing_Velocity_mps,"
        << "LOS_Rate_rad_s,Seeker_Look_Angle_deg,Missile_Speed_mps,Motor_Stage,Outcome\n";

    std::lock_guard<std::mutex> lock(entitiesMutex_);
    for (const auto& msl : activeMissiles_) {
        if (!msl) continue;
        for (const auto& pt : msl->getFlightRecorder()) {
            csv << std::fixed << std::setprecision(3) << pt.timestampSec << ","
                << pt.targetId << ","
                << pt.interceptorId << ","
                << std::setprecision(1) << pt.slantRangeM << ","
                << pt.closingVelocityMps << ","
                << std::setprecision(4) << pt.losRateRadS << ","
                << std::setprecision(2) << pt.seekerLookAngleDeg << ","
                << std::setprecision(1) << pt.missileSpeedMps << ","
                << pt.motorStage << ","
                << pt.outcome << "\n";
        }
    }

    csv.close();
    return true;
}

// ============================================================================
// Simulation Lifecycle
// ============================================================================

void SimulationEngine::startAsync(int wsPort) {
    if (isRunning_.exchange(true)) return;
    stopRequested_.store(false);

    wsBridge_ = std::make_unique<Bridge::WebSocketBridge>(wsPort);
    wsBridge_->setMessageCallback([this](const std::string& msg) {
        handleRawJsonCommand(msg);
    });
    wsBridge_->start();

    engineThread_ = std::thread([this]() {
        runBlocking(0.0);
    });
}

void SimulationEngine::runBlocking(double maxDurationSec) {
    isRunning_.store(true);
    stopRequested_.store(false);

    using Clock = std::chrono::steady_clock;
    auto startTime = Clock::now();
    auto lastPhysicsTick = startTime;
    auto lastRadarTick = startTime;
    auto lastTelemetryTick = startTime;
    auto lastJsonTick = startTime;

    constexpr auto PHYSICS_PERIOD   = std::chrono::milliseconds(10);  // 100 Hz
    constexpr auto RADAR_PERIOD     = std::chrono::milliseconds(50);  // 20 Hz
    constexpr auto TELEMETRY_PERIOD = std::chrono::milliseconds(100); // 10 Hz
    constexpr auto JSON_PERIOD      = std::chrono::milliseconds(20);  // 50 Hz Web Stream

    while (!stopRequested_.load()) {
        auto now = Clock::now();
        double elapsedSec = std::chrono::duration<double>(now - startTime).count();
        simTimeSec_.store(elapsedSec);

        if (maxDurationSec > 0.0 && elapsedSec >= maxDurationSec) {
            break;
        }

        // 1. Physics Tick (100 Hz)
        if (now - lastPhysicsTick >= PHYSICS_PERIOD) {
            double dt = std::chrono::duration<double>(now - lastPhysicsTick).count();
            physicsTick(std::min(dt, 0.05));
            lastPhysicsTick = now;
        }

        // 2. Radar Sweep Tick (20 Hz)
        if (now - lastRadarTick >= RADAR_PERIOD) {
            double dt = std::chrono::duration<double>(now - lastRadarTick).count();
            radarTick(dt);
            lastRadarTick = now;
        }

        // 3. Bus Telemetry Tick (10 Hz)
        if (now - lastTelemetryTick >= TELEMETRY_PERIOD) {
            busTelemetryTick();
            lastTelemetryTick = now;
        }

        // 4. JSON Telemetry Broadcast Tick (50 Hz / 20ms)
        if (now - lastJsonTick >= JSON_PERIOD) {
            telemetryJsonTick();
            lastJsonTick = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    isRunning_.store(false);
}

void SimulationEngine::stop() {
    stopRequested_.store(true);
    if (wsBridge_) {
        wsBridge_->stop();
    }
    if (engineThread_.joinable()) {
        engineThread_.join();
    }
    isRunning_.store(false);
}

} // namespace SDN::Engine
