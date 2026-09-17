/**
 * @file WebSocketBridge.cpp
 * @brief Production zero-dependency RFC 6455 WebSocket Server Implementation.
 */

#include "WebSocketBridge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <vector>

#if defined(_WIN32) || defined(_WIN64)
    #define PLATFORM_WINDOWS 1
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    typedef SOCKET SocketType;
    #define INVALID_SOCK INVALID_SOCKET
    #define CLOSE_SOCK(s) closesocket(s)
#else
    #define PLATFORM_WINDOWS 0
    #include <fcntl.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <sys/socket.h>
    #include <sys/types.h>
    #include <unistd.h>
    typedef int SocketType;
    #define INVALID_SOCK (-1)
    #define CLOSE_SOCK(s) ::close(s)
#endif

namespace SDN::Bridge {

namespace {

// ============================================================================
// SHA-1 Implementation (RFC 3174)
// ============================================================================
class SHA1 {
public:
    SHA1() { reset(); }

    void update(const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            buffer_[bufferIdx_++] = data[i];
            if (bufferIdx_ == 64) {
                processBlock();
                bufferIdx_ = 0;
            }
        }
        totalBytes_ += len;
    }

    void update(const std::string& str) {
        update(reinterpret_cast<const uint8_t*>(str.data()), str.length());
    }

    std::array<uint8_t, 20> digest() {
        uint64_t totalBits = totalBytes_ * 8;
        buffer_[bufferIdx_++] = 0x80;

        if (bufferIdx_ > 56) {
            while (bufferIdx_ < 64) buffer_[bufferIdx_++] = 0;
            processBlock();
            bufferIdx_ = 0;
        }

        while (bufferIdx_ < 56) buffer_[bufferIdx_++] = 0;

        for (int i = 7; i >= 0; --i) {
            buffer_[bufferIdx_++] = static_cast<uint8_t>((totalBits >> (i * 8)) & 0xFF);
        }
        processBlock();

        std::array<uint8_t, 20> out;
        for (int i = 0; i < 5; ++i) {
            out[i * 4 + 0] = static_cast<uint8_t>((h_[i] >> 24) & 0xFF);
            out[i * 4 + 1] = static_cast<uint8_t>((h_[i] >> 16) & 0xFF);
            out[i * 4 + 2] = static_cast<uint8_t>((h_[i] >> 8) & 0xFF);
            out[i * 4 + 3] = static_cast<uint8_t>(h_[i] & 0xFF);
        }
        return out;
    }

private:
    void reset() {
        h_[0] = 0x67452301;
        h_[1] = 0xEFCDAB89;
        h_[2] = 0x98BADCFE;
        h_[3] = 0x10325476;
        h_[4] = 0xC3D2E1F0;
        bufferIdx_ = 0;
        totalBytes_ = 0;
    }

    static uint32_t rotl(uint32_t val, uint32_t bits) {
        return (val << bits) | (val >> (32 - bits));
    }

    void processBlock() {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(buffer_[i * 4]) << 24) |
                   (static_cast<uint32_t>(buffer_[i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(buffer_[i * 4 + 2]) << 8) |
                   (static_cast<uint32_t>(buffer_[i * 4 + 3]));
        }
        for (int i = 16; i < 80; ++i) {
            w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }

        uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3], e = h_[4];

        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20) {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }
            uint32_t temp = rotl(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = rotl(b, 30);
            b = a;
            a = temp;
        }

        h_[0] += a;
        h_[1] += b;
        h_[2] += c;
        h_[3] += d;
        h_[4] += e;
    }

    uint32_t h_[5]{};
    uint8_t buffer_[64]{};
    size_t bufferIdx_{0};
    uint64_t totalBytes_{0};
};

// ============================================================================
// Base64 Encoding
// ============================================================================
std::string base64Encode(const uint8_t* data, size_t len) {
    static const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);

    for (size_t i = 0; i < len; i += 3) {
        uint32_t val = (data[i] << 16) |
                       ((i + 1 < len ? data[i + 1] : 0) << 8) |
                       (i + 2 < len ? data[i + 2] : 0);

        out.push_back(tbl[(val >> 18) & 0x3F]);
        out.push_back(tbl[(val >> 12) & 0x3F]);
        out.push_back(i + 1 < len ? tbl[(val >> 6) & 0x3F] : '=');
        out.push_back(i + 2 < len ? tbl[val & 0x3F] : '=');
    }
    return out;
}

std::string computeAcceptKey(const std::string& clientKey) {
    const std::string wsGuid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    SHA1 sha;
    sha.update(clientKey + wsGuid);
    auto hash = sha.digest();
    return base64Encode(hash.data(), hash.size());
}

} // anonymous namespace

// ============================================================================
// WebSocket Bridge Implementation
// ============================================================================

WebSocketBridge::WebSocketBridge(int port) : port_(port) {
#if PLATFORM_WINDOWS
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
}

WebSocketBridge::~WebSocketBridge() {
    stop();
#if PLATFORM_WINDOWS
    WSACleanup();
#endif
}

void WebSocketBridge::start() {
    if (isRunning_.exchange(true)) return;
    stopRequested_.store(false);

    serverThread_ = std::thread([this]() {
        serverLoop();
    });
}

void WebSocketBridge::stop() {
    stopRequested_.store(true);
    isRunning_.store(false);

    if (serverSocket_ != static_cast<uintptr_t>(INVALID_SOCK)) {
        CLOSE_SOCK(static_cast<SocketType>(serverSocket_));
        serverSocket_ = static_cast<uintptr_t>(INVALID_SOCK);
    }

    {
        std::lock_guard<std::mutex> lock(clientsMutex_);
        for (auto s : clientSockets_) {
            CLOSE_SOCK(static_cast<SocketType>(s));
        }
        clientSockets_.clear();
    }

    if (serverThread_.joinable()) {
        serverThread_.join();
    }
}

void WebSocketBridge::serverLoop() {
    SocketType sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCK) {
        isRunning_.store(false);
        return;
    }

    int opt = 1;
#if PLATFORM_WINDOWS
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
#else
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        CLOSE_SOCK(sock);
        isRunning_.store(false);
        return;
    }

    if (::listen(sock, 10) != 0) {
        CLOSE_SOCK(sock);
        isRunning_.store(false);
        return;
    }

    serverSocket_ = static_cast<uintptr_t>(sock);

    while (!stopRequested_.load()) {
        sockaddr_in clientAddr{};
        int clientLen = sizeof(clientAddr);
        SocketType clientSock = ::accept(sock, reinterpret_cast<sockaddr*>(&clientAddr), 
#if PLATFORM_WINDOWS
            &clientLen
#else
            reinterpret_cast<socklen_t*>(&clientLen)
#endif
        );

        if (clientSock == INVALID_SOCK) {
            if (stopRequested_.load()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        std::thread([this, clientSock]() {
            handleClient(static_cast<uintptr_t>(clientSock));
        }).detach();
    }
}

void WebSocketBridge::handleClient(uintptr_t sockVal) {
    SocketType sock = static_cast<SocketType>(sockVal);

    // 1. Perform HTTP WebSocket Handshake
    char buffer[4096];
    int bytesRead = ::recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (bytesRead <= 0) {
        CLOSE_SOCK(sock);
        return;
    }
    buffer[bytesRead] = '\0';
    std::string request(buffer, bytesRead);

    size_t keyPos = request.find("Sec-WebSocket-Key:");
    if (keyPos == std::string::npos) {
        CLOSE_SOCK(sock);
        return;
    }

    size_t start = request.find_first_not_of(" \t", keyPos + 18);
    size_t end = request.find("\r\n", start);
    std::string clientKey = request.substr(start, end - start);

    std::string acceptKey = computeAcceptKey(clientKey);

    std::ostringstream response;
    response << "HTTP/1.1 101 Switching Protocols\r\n"
             << "Upgrade: websocket\r\n"
             << "Connection: Upgrade\r\n"
             << "Sec-WebSocket-Accept: " << acceptKey << "\r\n\r\n";

    std::string respStr = response.str();
    ::send(sock, respStr.c_str(), static_cast<int>(respStr.length()), 0);

    // Register active client
    {
        std::lock_guard<std::mutex> lock(clientsMutex_);
        clientSockets_.push_back(sockVal);
    }

    // 2. Client Frame Processing Loop (RFC 6455)
    while (!stopRequested_.load()) {
        uint8_t header[2];
        int rec = ::recv(sock, reinterpret_cast<char*>(header), 2, 0);
        if (rec <= 0) break;

        uint8_t opcode = header[0] & 0x0F;
        bool masked = (header[1] & 0x80) != 0;
        uint64_t payloadLen = header[1] & 0x7F;

        if (opcode == 0x08) { // Close frame
            break;
        }

        if (payloadLen == 126) {
            uint8_t ext[2];
            ::recv(sock, reinterpret_cast<char*>(ext), 2, 0);
            payloadLen = (static_cast<uint64_t>(ext[0]) << 8) | ext[1];
        } else if (payloadLen == 127) {
            uint8_t ext[8];
            ::recv(sock, reinterpret_cast<char*>(ext), 8, 0);
            payloadLen = 0;
            for (int i = 0; i < 8; ++i) payloadLen = (payloadLen << 8) | ext[i];
        }

        uint8_t maskKey[4] = {0, 0, 0, 0};
        if (masked) {
            ::recv(sock, reinterpret_cast<char*>(maskKey), 4, 0);
        }

        std::vector<uint8_t> payload(payloadLen);
        size_t totalReceived = 0;
        while (totalReceived < payloadLen) {
            int r = ::recv(sock, reinterpret_cast<char*>(payload.data() + totalReceived), 
                           static_cast<int>(payloadLen - totalReceived), 0);
            if (r <= 0) break;
            totalReceived += r;
        }

        if (masked) {
            for (size_t i = 0; i < payloadLen; ++i) {
                payload[i] ^= maskKey[i % 4];
            }
        }

        if (opcode == 0x01) { // Text frame
            std::string text(payload.begin(), payload.end());
            std::lock_guard<std::mutex> lock(cbMutex_);
            if (callback_) {
                callback_(text);
            }
        }
    }

    // Deregister client
    {
        std::lock_guard<std::mutex> lock(clientsMutex_);
        clientSockets_.erase(std::remove(clientSockets_.begin(), clientSockets_.end(), sockVal), clientSockets_.end());
    }
    CLOSE_SOCK(sock);
}

void WebSocketBridge::broadcastText(const std::string& message) {
    if (!isRunning_.load() || message.empty()) return;

    // Construct WebSocket Text Frame (RFC 6455)
    std::vector<uint8_t> frame;
    frame.push_back(0x81); // FIN=1, Opcode=1 (Text)

    size_t len = message.length();
    if (len <= 125) {
        frame.push_back(static_cast<uint8_t>(len));
    } else if (len <= 65535) {
        frame.push_back(126);
        frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(len & 0xFF));
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; --i) {
            frame.push_back(static_cast<uint8_t>((len >> (i * 8)) & 0xFF));
        }
    }

    frame.insert(frame.end(), message.begin(), message.end());

    std::lock_guard<std::mutex> lock(clientsMutex_);
    for (auto it = clientSockets_.begin(); it != clientSockets_.end();) {
        SocketType s = static_cast<SocketType>(*it);
        int sent = ::send(s, reinterpret_cast<const char*>(frame.data()), static_cast<int>(frame.size()), 0);
        if (sent <= 0) {
            CLOSE_SOCK(s);
            it = clientSockets_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace SDN::Bridge
