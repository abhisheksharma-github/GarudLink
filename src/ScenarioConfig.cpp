/**
 * @file ScenarioConfig.cpp
 * @brief Implementation of declarative scenario configuration loader and JSON parser.
 */

#include "ScenarioConfig.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace SDN::Config {

namespace {

// Lightweight JSON Tokenizer & Parser Helpers
class MiniJson {
public:
    static std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(" \t\r\n");
        return str.substr(first, (last - first + 1));
    }

    static std::string getSubObject(const std::string& json, const std::string& key) {
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return "";

        size_t colon = json.find(':', pos + pattern.length());
        if (colon == std::string::npos) return "";

        size_t start = json.find('{', colon);
        if (start == std::string::npos) return "";

        int depth = 0;
        for (size_t i = start; i < json.length(); ++i) {
            if (json[i] == '{') depth++;
            else if (json[i] == '}') {
                depth--;
                if (depth == 0) {
                    return json.substr(start, i - start + 1);
                }
            }
        }
        return "";
    }

    static std::vector<std::string> getArrayObjects(const std::string& json, const std::string& key) {
        std::vector<std::string> items;
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return items;

        size_t colon = json.find(':', pos + pattern.length());
        if (colon == std::string::npos) return items;

        size_t start = json.find('[', colon);
        if (start == std::string::npos) return items;

        int arrayDepth = 0;
        int objDepth = 0;
        size_t objStart = std::string::npos;

        for (size_t i = start; i < json.length(); ++i) {
            char c = json[i];
            if (c == '[') {
                arrayDepth++;
            } else if (c == ']') {
                arrayDepth--;
                if (arrayDepth == 0) break;
            } else if (c == '{') {
                if (objDepth == 0) objStart = i;
                objDepth++;
            } else if (c == '}') {
                objDepth--;
                if (objDepth == 0 && objStart != std::string::npos) {
                    items.push_back(json.substr(objStart, i - objStart + 1));
                    objStart = std::string::npos;
                }
            }
        }
        return items;
    }

    static std::string getString(const std::string& json, const std::string& key, const std::string& defVal = "") {
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return defVal;

        size_t colon = json.find(':', pos + pattern.length());
        if (colon == std::string::npos) return defVal;

        size_t firstQuote = json.find('"', colon + 1);
        if (firstQuote == std::string::npos) return defVal;

        size_t secondQuote = json.find('"', firstQuote + 1);
        if (secondQuote == std::string::npos) return defVal;

        return json.substr(firstQuote + 1, secondQuote - firstQuote - 1);
    }

    static double getDouble(const std::string& json, const std::string& key, double defVal = 0.0) {
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return defVal;

        size_t colon = json.find(':', pos + pattern.length());
        if (colon == std::string::npos) return defVal;

        size_t start = json.find_first_of("0123456789-+.", colon + 1);
        if (start == std::string::npos) return defVal;

        size_t end = json.find_first_not_of("0123456789-+.", start);
        std::string valStr = json.substr(start, (end == std::string::npos ? json.length() : end) - start);

        try {
            return std::stod(valStr);
        } catch (...) {
            return defVal;
        }
    }

    static int getInt(const std::string& json, const std::string& key, int defVal = 0) {
        return static_cast<int>(getDouble(json, key, static_cast<double>(defVal)));
    }

    static bool getBool(const std::string& json, const std::string& key, bool defVal = false) {
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return defVal;

        size_t colon = json.find(':', pos + pattern.length());
        if (colon == std::string::npos) return defVal;

        size_t tPos = json.find("true", colon + 1);
        size_t fPos = json.find("false", colon + 1);

        if (tPos != std::string::npos && (fPos == std::string::npos || tPos < fPos)) return true;
        if (fPos != std::string::npos) return false;
        return defVal;
    }

    static Math::Vector3D getVector3D(const std::string& json, const std::string& key, const Math::Vector3D& defVal = {}) {
        std::string pattern = "\"" + key + "\"";
        size_t pos = json.find(pattern);
        if (pos == std::string::npos) return defVal;

        size_t openBracket = json.find('[', pos + pattern.length());
        if (openBracket == std::string::npos) return defVal;

        size_t closeBracket = json.find(']', openBracket);
        if (closeBracket == std::string::npos) return defVal;

        std::string arrStr = json.substr(openBracket + 1, closeBracket - openBracket - 1);
        std::stringstream ss(arrStr);
        std::string token;
        std::vector<double> vals;

        while (std::getline(ss, token, ',')) {
            token = trim(token);
            try {
                if (!token.empty()) vals.push_back(std::stod(token));
            } catch (...) {}
        }

        if (vals.size() >= 3) {
            return Math::Vector3D{vals[0], vals[1], vals[2]};
        }
        return defVal;
    }
};

} // anonymous namespace

ScenarioConfig ScenarioConfig::createDefault() {
    ScenarioConfig cfg;
    cfg.simulation.name = "Default Naval Air Defense Scenario";
    cfg.simulation.tickRateHz = 100;
    cfg.simulation.durationS = 300.0;
    cfg.simulation.seed = 42;

    cfg.ownship.id = "DDG-1000";
    cfg.ownship.position = Math::Vector3D{0.0, 0.0, -25.0};
    cfg.ownship.velocity = Math::Vector3D{0.0, 0.0, 0.0};

    cfg.ownship.radar.id = "RDR-01";
    cfg.ownship.radar.transmitterPowerW = 60000.0;
    cfg.ownship.radar.frequencyHz = 9.4e9;
    cfg.ownship.radar.antennaGainDbi = 38.0;
    cfg.ownship.radar.noiseFloorDbm = -100.0;
    cfg.ownship.radar.scanMode = "SURVEILLANCE_360";
    cfg.ownship.radar.scanRateDegS = 90.0;
    cfg.ownship.radar.azimuthBeamwidthDeg = 2.0;
    cfg.ownship.radar.elevationBeamwidthDeg = 30.0;

    cfg.defenseDoctrine.interceptorCount = 8;
    cfg.defenseDoctrine.doctrineMode = "AUTO_DEFENSE";
    cfg.defenseDoctrine.maxEngagementRangeM = 15000.0;
    cfg.defenseDoctrine.proportionalNavGain = 4.0;
    cfg.defenseDoctrine.maxGLimit = 35.0;
    cfg.defenseDoctrine.proximityFuseRadiusM = 12.0;

    TargetConfig tgt1;
    tgt1.id = "UAV-ALPHA";
    tgt1.type = "UAV";
    tgt1.spawnTimeS = 0.0;
    tgt1.initialPosition = Math::Vector3D{14142.0, 14142.0, -1500.0};
    tgt1.initialVelocity = Math::Vector3D{-127.3, -127.3, 0.0};
    tgt1.speedMps = 180.0;
    tgt1.rcsM2 = 0.5;
    tgt1.evasionProfile = "WEAVE_6G";
    tgt1.evasionFrequencyHz = 0.5;
    tgt1.evasionAmplitudeG = 6.0;
    tgt1.activeJamming = false;
    cfg.targets.push_back(tgt1);

    return cfg;
}

ScenarioConfig ScenarioConfig::loadFromFile(const std::string& filepath, std::string& outError) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        outError = "Could not open scenario configuration file: " + filepath;
        return createDefault();
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    return loadFromJsonString(buffer.str(), outError);
}

ScenarioConfig ScenarioConfig::loadFromJsonString(const std::string& json, std::string& outError) {
    ScenarioConfig cfg = createDefault();
    std::string trimmed = MiniJson::trim(json);
    if (trimmed.empty() || trimmed.front() != '{' || trimmed.back() != '}') {
        outError = "Malformed JSON syntax: root object must start with '{' and end with '}'";
        return cfg;
    }

    try {
        // 1. Parse Simulation Params
        std::string simObj = MiniJson::getSubObject(json, "simulation");
        if (!simObj.empty()) {
            cfg.simulation.name = MiniJson::getString(simObj, "name", cfg.simulation.name);
            cfg.simulation.tickRateHz = MiniJson::getInt(simObj, "tick_rate_hz", cfg.simulation.tickRateHz);
            cfg.simulation.durationS = MiniJson::getDouble(simObj, "duration_s", cfg.simulation.durationS);
            cfg.simulation.seed = static_cast<unsigned int>(MiniJson::getInt(simObj, "seed", cfg.simulation.seed));
        }

        // 2. Parse Ownship & Radar
        std::string ownObj = MiniJson::getSubObject(json, "ownship");
        if (!ownObj.empty()) {
            cfg.ownship.id = MiniJson::getString(ownObj, "id", cfg.ownship.id);
            cfg.ownship.position = MiniJson::getVector3D(ownObj, "position", cfg.ownship.position);
            cfg.ownship.velocity = MiniJson::getVector3D(ownObj, "velocity", cfg.ownship.velocity);

            std::string rdrObj = MiniJson::getSubObject(ownObj, "radar");
            if (!rdrObj.empty()) {
                cfg.ownship.radar.id = MiniJson::getString(rdrObj, "id", cfg.ownship.radar.id);
                cfg.ownship.radar.transmitterPowerW = MiniJson::getDouble(rdrObj, "transmitter_power_w", cfg.ownship.radar.transmitterPowerW);
                cfg.ownship.radar.frequencyHz = MiniJson::getDouble(rdrObj, "frequency_hz", cfg.ownship.radar.frequencyHz);
                cfg.ownship.radar.antennaGainDbi = MiniJson::getDouble(rdrObj, "antenna_gain_dbi", cfg.ownship.radar.antennaGainDbi);
                cfg.ownship.radar.noiseFloorDbm = MiniJson::getDouble(rdrObj, "noise_floor_dbm", cfg.ownship.radar.noiseFloorDbm);
                cfg.ownship.radar.scanMode = MiniJson::getString(rdrObj, "scan_mode", cfg.ownship.radar.scanMode);
                cfg.ownship.radar.scanRateDegS = MiniJson::getDouble(rdrObj, "scan_rate_deg_s", cfg.ownship.radar.scanRateDegS);
                cfg.ownship.radar.azimuthBeamwidthDeg = MiniJson::getDouble(rdrObj, "azimuth_beamwidth_deg", cfg.ownship.radar.azimuthBeamwidthDeg);
                cfg.ownship.radar.elevationBeamwidthDeg = MiniJson::getDouble(rdrObj, "elevation_beamwidth_deg", cfg.ownship.radar.elevationBeamwidthDeg);
            }
        }

        // 3. Parse Defense Doctrine
        std::string docObj = MiniJson::getSubObject(json, "defense_doctrine");
        if (!docObj.empty()) {
            cfg.defenseDoctrine.interceptorCount = MiniJson::getInt(docObj, "interceptor_count", cfg.defenseDoctrine.interceptorCount);
            cfg.defenseDoctrine.doctrineMode = MiniJson::getString(docObj, "doctrine_mode", cfg.defenseDoctrine.doctrineMode);
            cfg.defenseDoctrine.maxEngagementRangeM = MiniJson::getDouble(docObj, "max_engagement_range_m", cfg.defenseDoctrine.maxEngagementRangeM);
            cfg.defenseDoctrine.proportionalNavGain = MiniJson::getDouble(docObj, "proportional_nav_gain", cfg.defenseDoctrine.proportionalNavGain);
            cfg.defenseDoctrine.maxGLimit = MiniJson::getDouble(docObj, "max_g_limit", cfg.defenseDoctrine.maxGLimit);
            cfg.defenseDoctrine.proximityFuseRadiusM = MiniJson::getDouble(docObj, "proximity_fuse_radius_m", cfg.defenseDoctrine.proximityFuseRadiusM);
        }

        // 4. Parse Target Roster Array
        std::vector<std::string> targetObjs = MiniJson::getArrayObjects(json, "targets");
        if (!targetObjs.empty()) {
            cfg.targets.clear();
            for (const auto& tObj : targetObjs) {
                TargetConfig tgt;
                tgt.id = MiniJson::getString(tObj, "id", "TGT-" + std::to_string(cfg.targets.size() + 1));
                tgt.type = MiniJson::getString(tObj, "type", "UAV");
                tgt.spawnTimeS = MiniJson::getDouble(tObj, "spawn_time_s", 0.0);
                tgt.initialPosition = MiniJson::getVector3D(tObj, "initial_position", Math::Vector3D{15000.0, 15000.0, -1000.0});
                tgt.initialVelocity = MiniJson::getVector3D(tObj, "initial_velocity", Math::Vector3D{-100.0, -100.0, 0.0});
                tgt.speedMps = MiniJson::getDouble(tObj, "speed_mps", 180.0);
                tgt.rcsM2 = MiniJson::getDouble(tObj, "rcs_m2", 0.5);
                tgt.evasionProfile = MiniJson::getString(tObj, "evasion_profile", "NONE");
                tgt.evasionFrequencyHz = MiniJson::getDouble(tObj, "evasion_frequency_hz", 0.5);
                tgt.evasionAmplitudeG = MiniJson::getDouble(tObj, "evasion_amplitude_g", 6.0);
                tgt.activeJamming = MiniJson::getBool(tObj, "active_jamming", false);
                tgt.jammerPowerW = MiniJson::getDouble(tObj, "jammer_power_w", 0.0);
                cfg.targets.push_back(tgt);
            }
        }

        outError.clear();
    } catch (const std::exception& e) {
        outError = std::string("JSON parse exception: ") + e.what();
    }

    return cfg;
}

std::string ScenarioConfig::toJsonString() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "{\n";
    ss << "  \"simulation\": {\n";
    ss << "    \"name\": \"" << simulation.name << "\",\n";
    ss << "    \"tick_rate_hz\": " << simulation.tickRateHz << ",\n";
    ss << "    \"duration_s\": " << simulation.durationS << ",\n";
    ss << "    \"seed\": " << simulation.seed << "\n";
    ss << "  },\n";

    ss << "  \"ownship\": {\n";
    ss << "    \"id\": \"" << ownship.id << "\",\n";
    ss << "    \"position\": [" << ownship.position.x << ", " << ownship.position.y << ", " << ownship.position.z << "],\n";
    ss << "    \"velocity\": [" << ownship.velocity.x << ", " << ownship.velocity.y << ", " << ownship.velocity.z << "],\n";
    ss << "    \"radar\": {\n";
    ss << "      \"id\": \"" << ownship.radar.id << "\",\n";
    ss << "      \"transmitter_power_w\": " << ownship.radar.transmitterPowerW << ",\n";
    ss << "      \"frequency_hz\": " << ownship.radar.frequencyHz << ",\n";
    ss << "      \"antenna_gain_dbi\": " << ownship.radar.antennaGainDbi << ",\n";
    ss << "      \"scan_mode\": \"" << ownship.radar.scanMode << "\",\n";
    ss << "      \"scan_rate_deg_s\": " << ownship.radar.scanRateDegS << "\n";
    ss << "    }\n";
    ss << "  },\n";

    ss << "  \"defense_doctrine\": {\n";
    ss << "    \"interceptor_count\": " << defenseDoctrine.interceptorCount << ",\n";
    ss << "    \"doctrine_mode\": \"" << defenseDoctrine.doctrineMode << "\",\n";
    ss << "    \"max_engagement_range_m\": " << defenseDoctrine.maxEngagementRangeM << "\n";
    ss << "  },\n";

    ss << "  \"targets\": [\n";
    for (size_t i = 0; i < targets.size(); ++i) {
        const auto& t = targets[i];
        ss << "    {\n";
        ss << "      \"id\": \"" << t.id << "\",\n";
        ss << "      \"type\": \"" << t.type << "\",\n";
        ss << "      \"spawn_time_s\": " << t.spawnTimeS << ",\n";
        ss << "      \"initial_position\": [" << t.initialPosition.x << ", " << t.initialPosition.y << ", " << t.initialPosition.z << "],\n";
        ss << "      \"initial_velocity\": [" << t.initialVelocity.x << ", " << t.initialVelocity.y << ", " << t.initialVelocity.z << "],\n";
        ss << "      \"speed_mps\": " << t.speedMps << ",\n";
        ss << "      \"rcs_m2\": " << t.rcsM2 << ",\n";
        ss << "      \"evasion_profile\": \"" << t.evasionProfile << "\",\n";
        ss << "      \"active_jamming\": " << (t.activeJamming ? "true" : "false") << "\n";
        ss << "    }" << (i + 1 < targets.size() ? "," : "") << "\n";
    }
    ss << "  ]\n";
    ss << "}\n";

    return ss.str();
}

} // namespace SDN::Config
