/**
 * @file ShipDataBus.hpp
 * @brief Thread-safe asynchronous message bus and IEC 61162-1 / NMEA 0183 serialization protocol.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace SDN::Network {

// Common Naval/Maritime Talker Identifiers
namespace Talkers {
    inline constexpr const char* GPS_GNSS       = "GP"; // Global Positioning System
    inline constexpr const char* GYRO_COMPASS   = "HE"; // Heading, Gyro / Electronic
    inline constexpr const char* MAGNETIC_COMPASS = "HC"; // Heading, Magnetic Compass
    inline constexpr const char* RADAR          = "RA"; // Radar / Tactical Surface Search
    inline constexpr const char* DEPTH_SOUNDER  = "SD"; // Sounder, Depth
    inline constexpr const char* WEATHER_STATION= "WI"; // Weather Instruments (Anemometer, etc.)
    inline constexpr const char* AIS            = "AI"; // Automatic Identification System
    inline constexpr const char* COMBAT_SYSTEM  = "CS"; // Tactical Combat Management System
    inline constexpr const char* WILDCARD_ALL   = "*";  // Match all talkers
}

/**
 * @struct NetworkMessage
 * @brief Parsed IEC 61162-1 / NMEA 0183 sentence data structure.
 */
struct NetworkMessage {
    std::string talkerId{};                             ///< 2-character Talker ID (e.g. "GP", "HE", "RA")
    std::string sentenceType{};                         ///< 3-character Sentence Formatter (e.g. "GGA", "HDT", "TTM")
    std::vector<std::string> fields{};                  ///< Argument/data fields
    std::chrono::system_clock::time_point timestamp{};  ///< Reception/creation timestamp
    std::string rawPayload{};                           ///< Complete raw ASCII sentence string
    bool isValid{false};                                ///< Verification status (checksum & syntax validity)
    std::string errorMessage{};                         ///< Diagnostic information if isValid is false

    NetworkMessage() 
        : timestamp(std::chrono::system_clock::now()) {}

    NetworkMessage(std::string talker, std::string type, std::vector<std::string> args, std::string raw = "", bool valid = true)
        : talkerId(std::move(talker)),
          sentenceType(std::move(type)),
          fields(std::move(args)),
          timestamp(std::chrono::system_clock::now()),
          rawPayload(std::move(raw)),
          isValid(valid) {}

    /**
     * @brief Parses a raw NMEA/IEC 61162-1 string into a structured NetworkMessage.
     * @param rawSentence Full input sentence (e.g. "$GPGGA,123519,4807.038,N*47\r\n")
     * @return NetworkMessage Structured message with validity and error diagnostic flags.
     */
    [[nodiscard]] static NetworkMessage parse(const std::string& rawSentence);

    /**
     * @brief Serializes this message back to standard NMEA/IEC 61162-1 sentence with valid XOR checksum.
     */
    [[nodiscard]] std::string toFormattedSentence() const;
};

// ============================================================================
// NMEA 0183 / IEC 61162-1 Checksum & Formatting Functions
// ============================================================================

/**
 * @brief Calculates the 8-bit XOR checksum across character payload.
 * @param payload Characters between '$' and '*' (exclusive).
 * @return 8-bit checksum value.
 */
[[nodiscard]] uint8_t calculateChecksum(const std::string& payload) noexcept;

/**
 * @brief Validates the checksum and structure of a raw NMEA/IEC 61162-1 sentence.
 * @param sentence Full sentence string including delimiters.
 * @return true if formatting and checksum match exactly; false otherwise.
 */
[[nodiscard]] bool validateChecksum(const std::string& sentence) noexcept;

/**
 * @brief Formats a standard sentence with talker, message type, fields, and calculated checksum.
 * @param talkerId 2-char talker ID (e.g., "GP", "HE", "RA")
 * @param messageType 3-char message type (e.g., "GGA", "HDT", "MWV")
 * @param fields Vector of string field tokens
 * @return Formatted string conforming to "$<TALKER><TYPE>,<FIELDS...>*<XX>\r\n"
 */
[[nodiscard]] std::string formatSentence(
    const std::string& talkerId,
    const std::string& messageType,
    const std::vector<std::string>& fields
);

// ============================================================================
// Asynchronous Thread-Safe Ship Data Bus
// ============================================================================

/**
 * @class ShipDataBus
 * @brief Thread-safe asynchronous message bus for high-throughput naval tactical data exchange.
 */
class ShipDataBus {
public:
    using Callback = std::function<void(const NetworkMessage&)>;
    using SubscriptionId = uint64_t;

    explicit ShipDataBus(size_t maxQueueCapacity = 10000);
    ~ShipDataBus();

    // Disallow copy/move to prevent dangling background thread handles
    ShipDataBus(const ShipDataBus&) = delete;
    ShipDataBus& operator=(const ShipDataBus&) = delete;
    ShipDataBus(ShipDataBus&&) = delete;
    ShipDataBus& operator=(ShipDataBus&&) = delete;

    void start();
    void stop(bool drainQueue = true);

    SubscriptionId subscribe(const std::string& talkerId, Callback callback);
    bool unsubscribe(SubscriptionId subId);

    bool publish(const NetworkMessage& message);
    bool publishRaw(const std::string& rawSentence);

    [[nodiscard]] size_t queueSize() const;
    [[nodiscard]] size_t processedCount() const noexcept { return totalProcessedCount_.load(); }
    [[nodiscard]] size_t droppedCount() const noexcept { return totalDroppedCount_.load(); }
    [[nodiscard]] bool isRunning() const noexcept { return isRunning_.load(); }

    void clearStats() noexcept;

private:
    void dispatchLoop();

    size_t maxQueueCapacity_{10000};
    std::queue<NetworkMessage> messageQueue_;
    mutable std::mutex queueMutex_;
    std::condition_variable queueCv_;

    struct SubscriberRecord {
        SubscriptionId id;
        std::string talkerFilter;
        Callback callback;
    };

    mutable std::mutex subscriberMutex_;
    std::unordered_map<SubscriptionId, SubscriberRecord> subscribers_;
    SubscriptionId nextSubscriptionId_{1};

    std::atomic<bool> isRunning_{false};
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> drainRequested_{false};
    std::atomic<size_t> totalProcessedCount_{0};
    std::atomic<size_t> totalDroppedCount_{0};

    std::thread workerThread_;
};

} // namespace SDN::Network
