/**
 * @file SimulationEngine.hpp
 * @brief Production-grade multi-threaded tactical simulation engine, CMS Coordinator, and WebSocket bridge.
 */

#pragma once

#include "CommandProtocol.hpp"
#include "Entities.hpp"
#include "Math3D.hpp"
#include "ScenarioConfig.hpp"
#include "ScenarioManager.hpp"
#include "ShipDataBus.hpp"
#include "ThreatLibrary.hpp"
#include "WebSocketBridge.hpp"

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace SDN::Engine {

/**
 * @struct TrackState
 * @brief Track file entry maintained by the Combat Management System (CMS).
 */
struct TrackState {
    std::string trackId;
    double azimuthDeg{0.0};
    double elevationDeg{0.0};
    double rangeM{0.0};
    double snrDb{0.0};
    Math::Vector3D estimatedPos{0.0, 0.0, 0.0};
    Math::Vector3D estimatedVel{0.0, 0.0, 0.0};
    double lastUpdateSec{0.0};
    bool engagementAssigned{false};
    std::string assignedMissileId{};
};

/**
 * @class SimulationEngine
 * @brief High-frequency real-time simulation engine, Combat Management System (CMS), and tasking router.
 */
class SimulationEngine {
public:
    using TelemetryJsonCallback = std::function<void(const std::string&)>;

    explicit SimulationEngine(std::shared_ptr<Network::ShipDataBus> dataBus = nullptr);
    ~SimulationEngine();

    // Disallow copy/move
    SimulationEngine(const SimulationEngine&) = delete;
    SimulationEngine& operator=(const SimulationEngine&) = delete;

    // Declarative Scenario Loading
    bool loadScenarioConfig(const Config::ScenarioConfig& config);
    bool loadScenarioFile(const std::string& filepath, std::string& outError);
    bool loadScenarioJson(const std::string& json, std::string& outError);

    // Entity & Scenario Management
    void addEntity(std::shared_ptr<Entities::Entity> entity);
    void setPrimaryRadar(std::shared_ptr<Entities::RadarSystem> radar);
    [[nodiscard]] std::vector<std::shared_ptr<Entities::Entity>> getEntities() const;
    std::shared_ptr<Entities::Entity> findEntity(const std::string& id) const;

    // Operator Tasking & Command Protocol
    void handleCommand(const Protocol::OperatorCommand& cmd);
    void handleRawJsonCommand(const std::string& json);

    // Threat Spawning & Manual Injection
    std::shared_ptr<Entities::Drone> spawnThreat(Protocol::ThreatPreset preset, 
                                                 const Math::Vector3D& spawnPos,
                                                 const std::string& customId = "");

    void injectManualTarget(const ManualSpawnRequest& req);

    // Fire Control / CMS Configuration & Tasking
    void setEngagementRangeThreshold(double rangeM) noexcept { engagementRangeThresholdM_ = rangeM; }
    [[nodiscard]] double getEngagementRangeThreshold() const noexcept { return engagementRangeThresholdM_; }
    void setAutoEngagement(bool enable) noexcept { autoEngagementEnabled_ = enable; }
    [[nodiscard]] bool isAutoEngagementEnabled() const noexcept { return autoEngagementEnabled_; }

    void authorizeEngagement(const std::string& targetId);
    void launchMissileInterceptor(const TrackState& track);

    // Electronic Warfare (EW) & Radar Tasking
    void setRadarMode(Entities::RadarOperationalMode mode, double cueAzimuth = 0.0, 
                      double sectorWidth = 45.0, const std::string& targetId = "");
    void setTargetEvasion(const std::string& targetId, bool enable, double amplitudeG = 6.0, double freqHz = 0.5);
    void setTargetJamming(const std::string& targetId, bool enable, double jamPowerWatts = 500.0);

    // Simulation Lifecycle
    void startAsync(int wsPort = 8080);
    void runBlocking(double maxDurationSec = 0.0);
    void stop();
    [[nodiscard]] bool isRunning() const noexcept { return isRunning_.load(); }
    [[nodiscard]] double getSimulationTimeSec() const noexcept { return simTimeSec_.load(); }

    // After-Action Review (AAR) Flight Data Recorder
    bool exportAfterActionReportCsv(const std::string& filepath = "engagement_report.csv") const;

    // Telemetry Bridge Callbacks & JSON Export
    void setJsonTelemetryCallback(TelemetryJsonCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        jsonCallback_ = std::move(callback);
    }
    [[nodiscard]] std::string serializeStateJson() const;

    // Data Bus & Scenario Manager Access
    [[nodiscard]] std::shared_ptr<Network::ShipDataBus> getDataBus() const noexcept { return dataBus_; }
    [[nodiscard]] std::shared_ptr<ScenarioManager> getScenarioManager() const noexcept { return scenarioManager_; }

private:
    void initBusSubscriptions();
    void physicsTick(double dt);
    void radarTick(double dt);
    void busTelemetryTick();
    void telemetryJsonTick();

    void onRadarTrackMessage(const Network::NetworkMessage& msg);

    std::shared_ptr<Network::ShipDataBus> dataBus_;
    std::shared_ptr<ScenarioManager> scenarioManager_;
    std::shared_ptr<Entities::RadarSystem> primaryRadar_;
    std::unique_ptr<Bridge::WebSocketBridge> wsBridge_;

    mutable std::mutex entitiesMutex_;
    std::vector<std::shared_ptr<Entities::Entity>> entities_;
    std::vector<std::shared_ptr<Entities::GuidedMissile>> activeMissiles_;

    // CMS Fire Control State
    mutable std::mutex cmsMutex_;
    std::unordered_map<std::string, TrackState> trackTable_;
    double engagementRangeThresholdM_{15000.0};
    bool autoEngagementEnabled_{true};
    int remainingInterceptors_{8};
    uint32_t missileLaunchCounter_{1};
    uint32_t threatCounter_{1};

    // NMEA Log Ring Buffer
    mutable std::mutex logMutex_;
    std::deque<std::string> recentNmeaLogs_;
    static constexpr size_t MAX_LOG_ENTRIES = 25;

    // Multi-rate execution
    std::atomic<bool> isRunning_{false};
    std::atomic<bool> stopRequested_{false};
    std::atomic<double> simTimeSec_{0.0};
    std::thread engineThread_;

    mutable std::mutex callbackMutex_;
    TelemetryJsonCallback jsonCallback_;
};

} // namespace SDN::Engine
