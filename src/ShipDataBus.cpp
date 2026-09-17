/**
 * @file ShipDataBus.cpp
 * @brief Implementation of NMEA 0183 / IEC 61162-1 Serializer, Parser, and Thread-Safe Ship Data Bus.
 */

#include "ShipDataBus.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace SDN::Network {

namespace {

// Trims leading/trailing whitespace, carriage returns, and newlines
std::string trimString(const std::string& str) {
    size_t start = 0;
    while (start < str.size() && (std::isspace(static_cast<unsigned char>(str[start])) || str[start] == '\r' || str[start] == '\n')) {
        start++;
    }
    size_t end = str.size();
    while (end > start && (std::isspace(static_cast<unsigned char>(str[end - 1])) || str[end - 1] == '\r' || str[end - 1] == '\n')) {
        end--;
    }
    return str.substr(start, end - start);
}

// Converts a byte to 2-digit uppercase hex string
std::string byteToHex(uint8_t byte) {
    static constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
    std::string hex;
    hex.reserve(2);
    hex.push_back(HEX_DIGITS[(byte >> 4) & 0x0F]);
    hex.push_back(HEX_DIGITS[byte & 0x0F]);
    return hex;
}

// Converts 2 hex characters to uint8_t
bool hexToByte(char high, char low, uint8_t& outByte) noexcept {
    auto hexVal = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };

    const int h = hexVal(high);
    const int l = hexVal(low);
    if (h == -1 || l == -1) {
        return false;
    }
    outByte = static_cast<uint8_t>((h << 4) | l);
    return true;
}

} // anonymous namespace

// ============================================================================
// Checksum Calculation & Validation
// ============================================================================

uint8_t calculateChecksum(const std::string& payload) noexcept {
    uint8_t checksum = 0;
    for (const char ch : payload) {
        checksum ^= static_cast<uint8_t>(ch);
    }
    return checksum;
}

bool validateChecksum(const std::string& sentence) noexcept {
    const std::string trimmed = trimString(sentence);
    if (trimmed.size() < 4) {
        return false;
    }

    if (trimmed.front() != '$' && trimmed.front() != '!') {
        return false;
    }

    const size_t starPos = trimmed.rfind('*');
    if (starPos == std::string::npos || starPos + 2 >= trimmed.size()) {
        return false;
    }

    const std::string payload = trimmed.substr(1, starPos - 1);

    uint8_t expectedChecksum = 0;
    if (!hexToByte(trimmed[starPos + 1], trimmed[starPos + 2], expectedChecksum)) {
        return false;
    }

    const uint8_t actualChecksum = calculateChecksum(payload);
    return actualChecksum == expectedChecksum;
}

std::string formatSentence(
    const std::string& talkerId,
    const std::string& messageType,
    const std::vector<std::string>& fields
) {
    std::string payload;
    payload.reserve(64);
    payload.append(talkerId);
    payload.append(messageType);

    for (const auto& field : fields) {
        payload.push_back(',');
        payload.append(field);
    }

    const uint8_t checksum = calculateChecksum(payload);
    const std::string hexChk = byteToHex(checksum);

    std::string fullSentence;
    fullSentence.reserve(payload.size() + 6);
    fullSentence.push_back('$');
    fullSentence.append(payload);
    fullSentence.push_back('*');
    fullSentence.append(hexChk);
    fullSentence.append("\r\n");

    return fullSentence;
}

// ============================================================================
// NetworkMessage Parser & Serializer
// ============================================================================

NetworkMessage NetworkMessage::parse(const std::string& rawSentence) {
    NetworkMessage msg;
    msg.rawPayload = rawSentence;
    msg.timestamp = std::chrono::system_clock::now();

    const std::string trimmed = trimString(rawSentence);
    if (trimmed.empty()) {
        msg.isValid = false;
        msg.errorMessage = "Sentence is empty";
        return msg;
    }

    if (trimmed.front() != '$' && trimmed.front() != '!') {
        msg.isValid = false;
        msg.errorMessage = "Missing starting delimiter ($ or !)";
        return msg;
    }

    const size_t starPos = trimmed.rfind('*');
    if (starPos == std::string::npos) {
        msg.isValid = false;
        msg.errorMessage = "Missing checksum delimiter (*)";
        return msg;
    }

    if (starPos + 2 >= trimmed.size()) {
        msg.isValid = false;
        msg.errorMessage = "Incomplete checksum hex characters";
        return msg;
    }

    const std::string payload = trimmed.substr(1, starPos - 1);
    uint8_t expectedChecksum = 0;
    if (!hexToByte(trimmed[starPos + 1], trimmed[starPos + 2], expectedChecksum)) {
        msg.isValid = false;
        msg.errorMessage = "Invalid hexadecimal characters in checksum";
        return msg;
    }

    const uint8_t actualChecksum = calculateChecksum(payload);
    if (actualChecksum != expectedChecksum) {
        msg.isValid = false;
        msg.errorMessage = "Checksum verification failed";
        return msg;
    }

    const size_t firstComma = payload.find(',');
    std::string address;
    std::string dataSection;

    if (firstComma == std::string::npos) {
        address = payload;
    } else {
        address = payload.substr(0, firstComma);
        dataSection = payload.substr(firstComma + 1);
    }

    if (address.size() >= 2) {
        msg.talkerId = address.substr(0, 2);
        msg.sentenceType = address.substr(2);
    } else {
        msg.talkerId = address;
        msg.sentenceType = "";
    }

    msg.fields.clear();
    if (!dataSection.empty() || firstComma != std::string::npos) {
        std::stringstream ss(dataSection);
        std::string token;
        while (std::getline(ss, token, ',')) {
            msg.fields.push_back(token);
        }
        if (!dataSection.empty() && dataSection.back() == ',') {
            msg.fields.push_back("");
        }
    }

    msg.isValid = true;
    msg.errorMessage.clear();
    return msg;
}

std::string NetworkMessage::toFormattedSentence() const {
    if (isValid && !rawPayload.empty()) {
        std::string trimmed = trimString(rawPayload);
        trimmed.append("\r\n");
        return trimmed;
    }
    return formatSentence(talkerId, sentenceType, fields);
}

// ============================================================================
// ShipDataBus Implementation
// ============================================================================

ShipDataBus::ShipDataBus(size_t maxQueueCapacity)
    : maxQueueCapacity_(std::max<size_t>(100, maxQueueCapacity)) {
    start();
}

ShipDataBus::~ShipDataBus() {
    stop(true);
}

void ShipDataBus::start() {
    if (isRunning_.exchange(true)) {
        return;
    }
    stopRequested_.store(false);
    drainRequested_.store(false);

    workerThread_ = std::thread([this]() {
        dispatchLoop();
    });
}

void ShipDataBus::stop(bool drainQueue) {
    if (!isRunning_.load()) {
        return;
    }

    drainRequested_.store(drainQueue);
    stopRequested_.store(true);
    queueCv_.notify_all();

    if (workerThread_.joinable()) {
        workerThread_.join();
    }

    isRunning_.store(false);
}

ShipDataBus::SubscriptionId ShipDataBus::subscribe(const std::string& talkerId, Callback callback) {
    if (!callback) return 0;

    std::lock_guard<std::mutex> lock(subscriberMutex_);
    SubscriptionId id = nextSubscriptionId_++;
    subscribers_[id] = SubscriberRecord{id, talkerId, std::move(callback)};
    return id;
}

bool ShipDataBus::unsubscribe(SubscriptionId subId) {
    std::lock_guard<std::mutex> lock(subscriberMutex_);
    return subscribers_.erase(subId) > 0;
}

bool ShipDataBus::publish(const NetworkMessage& message) {
    if (!isRunning_.load() || stopRequested_.load()) {
        return false;
    }

    std::unique_lock<std::mutex> lock(queueMutex_);
    if (messageQueue_.size() >= maxQueueCapacity_) {
        totalDroppedCount_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    messageQueue_.push(message);
    lock.unlock();
    queueCv_.notify_one();
    return true;
}

bool ShipDataBus::publishRaw(const std::string& rawSentence) {
    NetworkMessage msg = NetworkMessage::parse(rawSentence);
    return publish(msg);
}

size_t ShipDataBus::queueSize() const {
    std::lock_guard<std::mutex> lock(queueMutex_);
    return messageQueue_.size();
}

void ShipDataBus::clearStats() noexcept {
    totalProcessedCount_.store(0);
    totalDroppedCount_.store(0);
}

void ShipDataBus::dispatchLoop() {
    while (true) {
        NetworkMessage msg;

        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCv_.wait(lock, [this]() {
                return !messageQueue_.empty() || stopRequested_.load();
            });

            if (messageQueue_.empty()) {
                if (stopRequested_.load()) {
                    break;
                }
                continue;
            }

            if (stopRequested_.load() && !drainRequested_.load()) {
                break;
            }

            msg = std::move(messageQueue_.front());
            messageQueue_.pop();
        }

        totalProcessedCount_.fetch_add(1, std::memory_order_relaxed);

        std::vector<Callback> matchingCallbacks;
        {
            std::lock_guard<std::mutex> subLock(subscriberMutex_);
            matchingCallbacks.reserve(subscribers_.size());

            for (const auto& [id, sub] : subscribers_) {
                if (sub.talkerFilter == Talkers::WILDCARD_ALL || 
                    sub.talkerFilter == msg.talkerId) {
                    matchingCallbacks.push_back(sub.callback);
                }
            }
        }

        for (const auto& cb : matchingCallbacks) {
            try {
                cb(msg);
            } catch (const std::exception& ex) {
                std::cerr << "[ShipDataBus] Subscriber exception: " << ex.what() << "\n";
            } catch (...) {
                std::cerr << "[ShipDataBus] Unknown subscriber exception.\n";
            }
        }
    }
}

} // namespace SDN::Network
