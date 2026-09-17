/**
 * @file ThreatLibrary.hpp
 * @brief Standardized tactical threat library and dynamic scenario factory.
 */

#pragma once

#include "Entities.hpp"
#include "Math3D.hpp"

#include <memory>
#include <string>
#include <vector>

namespace SDN::Threats {

class ThreatLibrary {
public:
    /**
     * @brief Target 1: Low-observable Recon Quad-UAV.
     * Characteristics:
     * - RCS: 0.02 m^2 (Micro-Doppler carbon-composite drone)
     * - Speed: 35 m/s (~68 knots)
     * - Altitude: 150m (NED Z = -150.0m)
     * - Navigation: Loitering waypoint perimeter route
     * - Fluctuation: Swerling Case 1 (Slow Rayleigh fading)
     */
    static std::shared_ptr<Entities::Drone> createReconQuadUav(
        const std::string& id = "UAV-RECON-01",
        const Math::Vector3D& spawnPos = {12000.0, 10000.0, -150.0}
    ) {
        auto uav = std::make_shared<Entities::Drone>(
            id,
            Entities::Classification::HOSTILE,
            spawnPos,
            35.0,  // 35 m/s
            0.02,  // 0.02 m^2 RCS
            18.0,  // 18 kg
            "RECON_QUAD_UAV",
            Entities::SwerlingModel::SWERLING_1
        );

        // Circular / Box Loiter Waypoints
        uav->addWaypoint({spawnPos.x - 3000.0, spawnPos.y + 2000.0, -150.0});
        uav->addWaypoint({spawnPos.x - 5000.0, spawnPos.y - 3000.0, -150.0});
        uav->addWaypoint({spawnPos.x - 2000.0, spawnPos.y - 5000.0, -150.0});
        uav->addWaypoint(spawnPos);
        uav->setTurnRateLimitDegPerSec(25.0);

        return uav;
    }

    /**
     * @brief Target 2: High-speed Kamikaze Drone swarm member.
     * Characteristics:
     * - RCS: 0.05 m^2 (Delta-wing loitering munition)
     * - Speed: 70 m/s (~136 knots)
     * - Altitude: 300m (NED Z = -300.0m)
     * - Navigation: Inbound direct strike with high-G sinusoidal evasive weave (6G @ 0.6 Hz)
     * - Fluctuation: Swerling Case 1
     */
    static std::shared_ptr<Entities::Drone> createKamikazeDrone(
        const std::string& id = "DRONE-KAMI-01",
        const Math::Vector3D& spawnPos = {18000.0, 14000.0, -300.0}
    ) {
        auto drone = std::make_shared<Entities::Drone>(
            id,
            Entities::Classification::HOSTILE,
            spawnPos,
            70.0,  // 70 m/s
            0.05,  // 0.05 m^2 RCS
            85.0,  // 85 kg
            "KAMIKAZE_SWARM",
            Entities::SwerlingModel::SWERLING_1
        );

        drone->addWaypoint({0.0, 0.0, -10.0}); // Attack ship waterline
        drone->setEvasiveMode(true);
        drone->setEvasiveParameters(0.6, 6.0); // 0.6 Hz, 6G weave

        return drone;
    }

    /**
     * @brief Target 3: Supersonic Sea-Skimming Anti-Ship Missile (ASM).
     * Characteristics:
     * - RCS: 0.15 m^2
     * - Speed: 750 m/s (Mach 2.2 Supersonic Dash)
     * - Altitude: 10m (NED Z = -10.0m, sea-skimming horizon hide)
     * - Navigation: Terminal pop-up/weave sea-skimming profile
     * - Fluctuation: Swerling Case 3 (Dominant body scatterer)
     */
    static std::shared_ptr<Entities::Drone> createSeaSkimmerAsm(
        const std::string& id = "ASM-VAMPIRE-01",
        const Math::Vector3D& spawnPos = {25000.0, 20000.0, -10.0}
    ) {
        auto asmMissile = std::make_shared<Entities::Drone>(
            id,
            Entities::Classification::HOSTILE,
            spawnPos,
            750.0, // Mach 2.2 (750 m/s)
            0.15,  // 0.15 m^2 RCS
            650.0, // 650 kg
            "SEA_SKIMMER_ASM",
            Entities::SwerlingModel::SWERLING_3
        );

        asmMissile->addWaypoint({0.0, 0.0, -8.0}); // Direct ship strike
        asmMissile->setTurnRateLimitDegPerSec(15.0);
        asmMissile->setEvasiveMode(true);
        asmMissile->setEvasiveParameters(0.8, 8.0); // 8G terminal weave

        return asmMissile;
    }
};

} // namespace SDN::Threats
