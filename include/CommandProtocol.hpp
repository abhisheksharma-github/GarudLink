/**
 * @file CommandProtocol.hpp
 * @brief Operator command protocol, JSON serialization/deserialization, and mission tasking schemas.
 */

#pragma once

#include "Math3D.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace SDN::Protocol {

/**
 * @enum CommandType
 * @brief Supported real-time operator tasking and simulation commands.
 */
enum class CommandType {
    SPAWN_TARGET,
    AUTHORIZE_INTERCEPT,
    SET_RADAR_MODE,
    TRIGGER_EVASION,
    DEPLOY_JAMMING,
    SET_AUTO_ENGAGE,
    SET_RANGE_THRESHOLD,
    RESET_SCENARIO,
    LOAD_SCENARIO,
    UNKNOWN
};

/**
 * @enum ThreatPreset
 * @brief Standardized tactical threat templates.
 */
enum class ThreatPreset {
    RECON_QUAD_UAV,
    KAMIKAZE_SWARM,
    SEA_SKIMMER_ASM,
    CUSTOM
};

/**
 * @enum RadarOperationalMode
 * @brief Operational scanning regimes of the naval sensor array.
 */
enum class RadarOperationalMode {
    SURVEILLANCE_360,
    SECTOR_CUE,
    STT_TRACK_LOCK
};

[[nodiscard]] inline std::string radarModeToString(RadarOperationalMode mode) {
    switch (mode) {
        case RadarOperationalMode::SURVEILLANCE_360: return "SURVEILLANCE_360";
        case RadarOperationalMode::SECTOR_CUE:       return "SECTOR_CUE";
        case RadarOperationalMode::STT_TRACK_LOCK:   return "STT_TRACK_LOCK";
    }
    return "SURVEILLANCE_360";
}

[[nodiscard]] inline RadarOperationalMode stringToRadarMode(const std::string& sv) {
    if (sv == "SECTOR_CUE") return RadarOperationalMode::SECTOR_CUE;
    if (sv == "STT_TRACK_LOCK") return RadarOperationalMode::STT_TRACK_LOCK;
    return RadarOperationalMode::SURVEILLANCE_360;
}

/**
 * @struct OperatorCommand
 * @brief Parsed command packet received from WebSocket or Console.
 */
struct OperatorCommand {
    CommandType type{CommandType::UNKNOWN};
    
    // Threat Spawning Parameters
    ThreatPreset threatPreset{ThreatPreset::CUSTOM};
    std::string customId;
    std::string customType{"UAV"};
    Math::Vector3D spawnPosition{15000.0, 15000.0, -1500.0};
    Math::Vector3D initialVelocity{-120.0, -120.0, 0.0};
    double speedMps{180.0};
    double rcsM2{0.5};
    double altitudeM{1500.0};
    
    // Scenario Load Parameters
    std::string scenarioPath;
    std::string scenarioJsonContent;
    
    // Fire Control & Engagement Parameters
    std::string targetId;
    bool autoEngage{true};
    double rangeThresholdM{15000.0};
    
    // Radar Tasking Parameters
    RadarOperationalMode radarMode{RadarOperationalMode::SURVEILLANCE_360};
    double cueAzimuthDeg{0.0};
    double sectorWidthDeg{45.0};
    
    // Electronic Warfare (EW) Parameters
    bool evasionEnabled{false};
    double evasionAmplitudeG{6.0};
    double evasionFrequencyHz{0.5};
    bool jammingEnabled{false};
    double jammerPowerWatts{500.0};
};

/**
 * @class JsonHelper
 * @brief Lightweight, zero-dependency JSON utility for extracting fields and building responses.
 */
class JsonHelper {
public:
    static std::string extractString(const std::string& json, const std::string& key) {
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return "";

        size_t colon = json.find(':', pos + pattern.length());
        if (colon == std::string::npos) return "";

        size_t firstQuote = json.find('"', colon + 1);
        if (firstQuote == std::string::npos) return "";

        size_t secondQuote = json.find('"', firstQuote + 1);
        if (secondQuote == std::string::npos) return "";

        return json.substr(firstQuote + 1, secondQuote - firstQuote - 1);
    }

    static double extractDouble(const std::string& json, const std::string& key, double defaultVal = 0.0) {
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return defaultVal;

        size_t colon = json.find(':', pos + pattern.length());
        if (colon == std::string::npos) return defaultVal;

        size_t start = json.find_first_of("0123456789-+.", colon + 1);
        if (start == std::string::npos) return defaultVal;

        size_t end = json.find_first_not_of("0123456789-+.", start);
        std::string valStr = json.substr(start, (end == std::string::npos ? json.length() : end) - start);

        try {
            return std::stod(valStr);
        } catch (...) {
            return defaultVal;
        }
    }

    static bool extractBool(const std::string& json, const std::string& key, bool defaultVal = false) {
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return defaultVal;

        size_t colon = json.find(':', pos + pattern.length());
        if (colon == std::string::npos) return defaultVal;

        size_t tPos = json.find("true", colon + 1);
        size_t fPos = json.find("false", colon + 1);

        if (tPos != std::string::npos && (fPos == std::string::npos || tPos < fPos)) {
            return true;
        }
        if (fPos != std::string::npos) {
            return false;
        }
        return defaultVal;
    }

    static OperatorCommand parseCommand(const std::string& json) {
        OperatorCommand cmd;
        std::string cmdStr = extractString(json, "command");

        if (cmdStr == "SPAWN_TARGET") {
            cmd.type = CommandType::SPAWN_TARGET;
            std::string threat = extractString(json, "threat_type");
            if (threat == "QUAD_UAV" || threat == "RECON_QUAD_UAV") cmd.threatPreset = ThreatPreset::RECON_QUAD_UAV;
            else if (threat == "KAMIKAZE" || threat == "KAMIKAZE_SWARM") cmd.threatPreset = ThreatPreset::KAMIKAZE_SWARM;
            else if (threat == "SEA_SKIMMER_ASM" || threat == "ASM") cmd.threatPreset = ThreatPreset::SEA_SKIMMER_ASM;
            else cmd.threatPreset = ThreatPreset::CUSTOM;

            cmd.customId = extractString(json, "id");
            cmd.spawnPosition.x = extractDouble(json, "x", 15000.0);
            cmd.spawnPosition.y = extractDouble(json, "y", 15000.0);
            cmd.altitudeM = extractDouble(json, "altitude_m", 1500.0);
            cmd.spawnPosition.z = -std::abs(cmd.altitudeM); // NED convention (down is positive)
            cmd.speedMps = extractDouble(json, "speed_mps", 180.0);
            cmd.rcsM2 = extractDouble(json, "rcs_m2", 0.5);
            cmd.evasionEnabled = extractBool(json, "evasive", false);
            cmd.evasionAmplitudeG = extractDouble(json, "evasion_amplitude_g", 6.0);
            cmd.evasionFrequencyHz = extractDouble(json, "evasion_freq_hz", 0.5);
            cmd.jammingEnabled = extractBool(json, "jamming", false);
            cmd.jammerPowerWatts = extractDouble(json, "jam_power_w", 500.0);
        }
        else if (cmdStr == "AUTHORIZE_INTERCEPT") {
            cmd.type = CommandType::AUTHORIZE_INTERCEPT;
            cmd.targetId = extractString(json, "target_id");
        }
        else if (cmdStr == "SET_RADAR_MODE") {
            cmd.type = CommandType::SET_RADAR_MODE;
            cmd.radarMode = stringToRadarMode(extractString(json, "mode"));
            cmd.cueAzimuthDeg = extractDouble(json, "cue_azimuth", 0.0);
            cmd.sectorWidthDeg = extractDouble(json, "sector_width_deg", 45.0);
            cmd.targetId = extractString(json, "target_id");
        }
        else if (cmdStr == "TRIGGER_EVASION") {
            cmd.type = CommandType::TRIGGER_EVASION;
            cmd.targetId = extractString(json, "target_id");
            cmd.evasionEnabled = extractBool(json, "enable", true);
            cmd.evasionAmplitudeG = extractDouble(json, "amplitude_g", 6.0);
            cmd.evasionFrequencyHz = extractDouble(json, "freq_hz", 0.5);
        }
        else if (cmdStr == "DEPLOY_JAMMING") {
            cmd.type = CommandType::DEPLOY_JAMMING;
            cmd.targetId = extractString(json, "target_id");
            cmd.jammingEnabled = extractBool(json, "enable", true);
            cmd.jammerPowerWatts = extractDouble(json, "jam_power_w", 500.0);
        }
        else if (cmdStr == "SET_AUTO_ENGAGE") {
            cmd.type = CommandType::SET_AUTO_ENGAGE;
            cmd.autoEngage = extractBool(json, "enable", true);
        }
        else if (cmdStr == "SET_RANGE_THRESHOLD") {
            cmd.type = CommandType::SET_RANGE_THRESHOLD;
            cmd.rangeThresholdM = extractDouble(json, "threshold_m", 15000.0);
        }
        else if (cmdStr == "RESET_SCENARIO") {
            cmd.type = CommandType::RESET_SCENARIO;
        }
        else if (cmdStr == "LOAD_SCENARIO") {
            cmd.type = CommandType::LOAD_SCENARIO;
            cmd.scenarioPath = extractString(json, "filepath");
            cmd.scenarioJsonContent = extractString(json, "config_json");
        }

        return cmd;
    }
};

} // namespace SDN::Protocol
