
#include "EpuckHardware.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <iostream>
#include <utility>

// Robot reply packets: 1-byte header, then the payload.
//   0x01 = camera image (160x120 RGB565), 0x02 = sensors, 0x03 = empty ack
static constexpr size_t kSensorsPacketSize = 104;
static constexpr size_t kImagePacketSize = 160 * 120 * 2;

static bool recvAll(int socket, uint8_t* buffer, size_t length) {
    size_t received = 0;

    while (received < length) {
        ssize_t n = ::recv(socket, buffer + received, length - received, 0);

        if (n <= 0) {
            std::cerr << "Recv failed. Bytes received: " << received
                      << " of " << length << ", recv result: " << n;
            if (n < 0) std::cerr << ", error: " << std::strerror(errno);
            std::cerr << "\n";
            return false;
        }

        received += static_cast<size_t>(n);
    }

    return true;
}

// Reads reply packets until one with the wanted header arrives.
// Packets the caller did not ask for (an extra sensor packet or an
// image) are read and dropped so the stream stays in sync.
static bool recvPacket(int socket, uint8_t wantedHeader,
                       std::vector<uint8_t>& payload) {
    for (int packets = 0; packets < 4; ++packets) {
        uint8_t header = 0;
        if (!recvAll(socket, &header, 1)) return false;

        size_t size = 0;
        if (header == 0x01) size = kImagePacketSize;
        else if (header == 0x02) size = kSensorsPacketSize;
        else if (header == 0x03) size = 0;
        else {
            std::cerr << "Unknown reply header: 0x" << std::hex
                      << static_cast<int>(header) << std::dec << "\n";
            return false;
        }

        payload.assign(size, 0);
        if (size > 0 && !recvAll(socket, payload.data(), size)) return false;

        if (header == wantedHeader) return true;
    }

    std::cerr << "Wanted reply 0x" << std::hex << static_cast<int>(wantedHeader)
              << std::dec << " did not arrive\n";
    return false;
}

EpuckHardware::EpuckHardware(std::string ip, int port)
    : ip_(std::move(ip)), port_(port) {}

bool EpuckHardware::connect() {
    disconnect();

    socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_ < 0) {
        std::cerr << "Socket creation failed\n";
        return false;
    }

    timeval timeout{};
    timeout.tv_sec = 10;

    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO,
               &timeout, sizeof(timeout));
    setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO,
               &timeout, sizeof(timeout));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port_));

    if (inet_pton(AF_INET, ip_.c_str(), &address.sin_addr) != 1) {
        std::cerr << "Invalid robot IP address\n";
        disconnect();
        return false;
    }

    if (::connect(socket_,
                  reinterpret_cast<sockaddr*>(&address),
                  sizeof(address)) != 0) {
        std::cerr << "Connection failed: "
                  << std::strerror(errno) << "\n";
        disconnect();
        return false;
    }

    std::cout << "ROBOT CONNECTED: "
              << ip_ << ":" << port_ << "\n";

    return true;
}

bool EpuckHardware::readSensors(EpuckSensors& sensors) {
    return sendRequest() && receiveSensors(sensors);
}

// One 21-byte packet carries the motor speeds AND the sensor request,
// so reading sensors never resets the wheels to zero.
bool EpuckHardware::sendRequest() {
    if (socket_ < 0) {
        std::cerr << "Sensor error: robot not connected\n";
        return false;
    }

    uint8_t request[21] = {};
    request[0] = 0x80;
    request[1] = 0x02;
    request[2] = 0x00;
    request[3] = static_cast<uint8_t>(left_);
    request[4] = static_cast<uint8_t>(static_cast<uint16_t>(left_) >> 8);
    request[5] = static_cast<uint8_t>(right_);
    request[6] = static_cast<uint8_t>(static_cast<uint16_t>(right_) >> 8);

    size_t sent = 0;

    while (sent < sizeof(request)) {
        ssize_t n = ::send(
            socket_,
            request + sent,
            sizeof(request) - sent,
            0
        );

        if (n <= 0) {
            std::cerr << "Sensor request send failed: "
                      << std::strerror(errno) << "\n";
            return false;
        }

        sent += static_cast<size_t>(n);
    }

    return true;
}

bool EpuckHardware::receiveSensors(EpuckSensors& sensors) {
    sensors = EpuckSensors{};

    if (socket_ < 0) return false;

    std::vector<uint8_t> payload;
    if (!recvPacket(socket_, 0x02, payload)) return false;

    // Proximity values start at payload byte 37 (byte 38 counting the header).
    for (size_t i = 0; i < 8; ++i) {
        size_t offset = 37 + 2 * i;

        sensors.proximity[i] =
            static_cast<uint16_t>(payload[offset]) |
            (static_cast<uint16_t>(payload[offset + 1]) << 8);
    }

    sensors.valid = true;
    return true;
}

void EpuckHardware::disconnect() {
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
}

// Stores the speeds; they are sent with the next sensor request.
bool EpuckHardware::setWheelSpeeds(
    int16_t left,
    int16_t right
) {
    if (socket_ < 0) return false;

    // SAFETY: motors enabled for physical testing, speed limited.
    // To lock the motors again, swap these two lines back:
    // if (left != 0 || right != 0) return false;
    if (left > 400 || left < -400 || right > 400 || right < -400) return false;

    left_ = left;
    right_ = right;
    return true;
}

// Sends zero speed immediately.
bool EpuckHardware::stop() {
    left_ = 0;
    right_ = 0;
    EpuckSensors ignored;
    return readSensors(ignored);
}

// Requests one camera image (request byte 0x01). Current wheel speeds
// are sent in the same packet, so call stop() first for a sharp picture.
bool EpuckHardware::captureImage(std::vector<uint8_t>& rgb565) {
    if (socket_ < 0) return false;

    uint8_t request[21] = {};
    request[0] = 0x80;
    request[1] = 0x01;
    request[3] = static_cast<uint8_t>(left_);
    request[4] = static_cast<uint8_t>(static_cast<uint16_t>(left_) >> 8);
    request[5] = static_cast<uint8_t>(right_);
    request[6] = static_cast<uint8_t>(static_cast<uint16_t>(right_) >> 8);

    size_t sent = 0;
    while (sent < sizeof(request)) {
        ssize_t n = ::send(socket_, request + sent, sizeof(request) - sent, 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }

    // The robot may send a sensor packet before the image; recvPacket skips it.
    return recvPacket(socket_, 0x01, rgb565);
}

bool EpuckHardware::saveImageBmp(const std::string& path,
                                 const std::vector<uint8_t>& img) {
    const int w = 160, h = 120;
    if (img.size() != static_cast<size_t>(w * h * 2)) return false;

    const uint32_t dataSize = w * h * 3, fileSize = 54 + dataSize;
    uint8_t hdr[54] = {'B', 'M'};
    auto put32 = [&](int off, uint32_t v) {
        for (int i = 0; i < 4; ++i) hdr[off + i] = (v >> (8 * i)) & 0xFF;
    };
    put32(2, fileSize); put32(10, 54); put32(14, 40);
    put32(18, w); put32(22, h);
    hdr[26] = 1; hdr[28] = 24;
    put32(34, dataSize);

    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<char*>(hdr), 54);
    for (int y = h - 1; y >= 0; --y) {  // BMP rows go bottom-up
        for (int x = 0; x < w; ++x) {
            // RGB565, high byte first.
            uint8_t hi = img[2 * (y * w + x)], lo = img[2 * (y * w + x) + 1];
            uint8_t px[3] = {
                static_cast<uint8_t>((lo & 0x1F) << 3),
                static_cast<uint8_t>((hi << 5) | ((lo & 0xE0) >> 3)),
                static_cast<uint8_t>(hi & 0xF8)
            };
            f.write(reinterpret_cast<char*>(px), 3);
        }
    }
    return static_cast<bool>(f);
}
