/**
 * @file WebSocketBridge.hpp
 * @brief Zero-dependency RFC 6455 WebSocket server for real-time telemetry streaming and operator tasking.
 */

#pragma once

#include "CommandProtocol.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace SDN::Bridge {

class WebSocketBridge {
public:
    using MessageCallback = std::function<void(const std::string&)>;

    explicit WebSocketBridge(int port = 8080);
    ~WebSocketBridge();

    // Disallow copy/move
    WebSocketBridge(const WebSocketBridge&) = delete;
    WebSocketBridge& operator=(const WebSocketBridge&) = delete;

    void start();
    void stop();
    [[nodiscard]] bool isRunning() const noexcept { return isRunning_.load(); }
    [[nodiscard]] int getPort() const noexcept { return port_; }

    void broadcastText(const std::string& message);
    void setMessageCallback(MessageCallback cb) {
        std::lock_guard<std::mutex> lock(cbMutex_);
        callback_ = std::move(cb);
    }

private:
    void serverLoop();
    void handleClient(uintptr_t clientSocket);

    int port_{8080};
    std::atomic<bool> isRunning_{false};
    std::atomic<bool> stopRequested_{false};
    uintptr_t serverSocket_{~0ULL}; // invalid socket

    std::thread serverThread_;
    std::mutex clientsMutex_;
    std::vector<uintptr_t> clientSockets_;

    std::mutex cbMutex_;
    MessageCallback callback_;
};

} // namespace SDN::Bridge
