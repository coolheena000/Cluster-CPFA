// Standalone e-puck2 WiFi camera test (does not modify project code).
// Requests a QQVGA (160x120 RGB565) image and saves it as a BMP.
// Usage: ./test_epuck_camera [ip] [output.bmp]
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

static const int W = 160, H = 120, IMAGE_SIZE = W * H * 2, SENSORS_SIZE = 104;

static bool sendAll(int s, const uint8_t* buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(s, buf + sent, len - sent, 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

static bool recvAll(int s, uint8_t* buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = ::recv(s, buf + got, len - got, 0);
        if (n <= 0) {
            std::cerr << "recv failed after " << got << "/" << len << " bytes";
            if (n < 0) std::cerr << ": " << std::strerror(errno);
            std::cerr << "\n";
            return false;
        }
        got += static_cast<size_t>(n);
    }
    return true;
}

static bool saveBmp(const std::string& path, const std::vector<uint8_t>& img) {
    const int rowSize = W * 3;  // 480, already a multiple of 4
    const uint32_t dataSize = rowSize * H, fileSize = 54 + dataSize;
    uint8_t hdr[54] = {'B', 'M'};
    auto put32 = [&](int off, uint32_t v) { for (int i = 0; i < 4; ++i) hdr[off + i] = (v >> (8 * i)) & 0xFF; };
    put32(2, fileSize); put32(10, 54); put32(14, 40);
    put32(18, W); put32(22, H);
    hdr[26] = 1; hdr[28] = 24;
    put32(34, dataSize);

    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<char*>(hdr), 54);
    for (int y = H - 1; y >= 0; --y) {  // BMP rows go bottom-up
        for (int x = 0; x < W; ++x) {
            // RGB565, big-endian (high byte first), as in GCtronic's PC tools.
            uint8_t hi = img[2 * (y * W + x)], lo = img[2 * (y * W + x) + 1];
            uint8_t r = hi & 0xF8;
            uint8_t g = static_cast<uint8_t>((hi << 5) | ((lo & 0xE0) >> 3));
            uint8_t b = static_cast<uint8_t>((lo & 0x1F) << 3);
            uint8_t px[3] = {b, g, r};
            f.write(reinterpret_cast<char*>(px), 3);
        }
    }
    return static_cast<bool>(f);
}

int main(int argc, char** argv) {
    std::string ip = argc > 1 ? argv[1] : "172.20.10.2";
    std::string out = argc > 2 ? argv[2] : "epuck_camera.bmp";

    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    timeval tv{}; tv.tv_sec = 10;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(1000);
    inet_pton(AF_INET, ip.c_str(), &a.sin_addr);
    if (::connect(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
        std::cerr << "Connection failed: " << std::strerror(errno) << "\n";
        return 1;
    }
    std::cout << "ROBOT CONNECTED: " << ip << ":1000\n";

    // Same 21-byte packet as EpuckHardware, but request = 0x01 (camera only).
    // Motor speeds and LEDs stay zero.
    uint8_t cmd[21] = {};
    cmd[0] = 0x80;
    cmd[1] = 0x01;
    std::vector<uint8_t> img(IMAGE_SIZE);
    bool gotImage = false;

    // The first frame after enabling the camera can be dark; grab a few.
    for (int frame = 0; frame < 3; ++frame) {
        if (!sendAll(s, cmd, sizeof(cmd))) { std::cerr << "send failed\n"; break; }
        uint8_t header = 0;
        if (!recvAll(s, &header, 1)) break;
        if (header == 0x01) {
            if (!recvAll(s, img.data(), img.size())) break;
            gotImage = true;
            std::cout << "Frame " << frame + 1 << ": image received (" << IMAGE_SIZE << " bytes)\n";
        } else if (header == 0x02) {
            std::vector<uint8_t> skip(SENSORS_SIZE);
            if (!recvAll(s, skip.data(), skip.size())) break;
            std::cout << "Got sensor packet instead of image\n";
        } else {
            std::cout << "Got header 0x" << std::hex << int(header) << std::dec << "\n";
        }
    }
    ::close(s);

    if (!gotImage) {
        std::cerr << "CAMERA TEST FAILED: no image received\n";
        return 1;
    }
    if (!saveBmp(out, img)) {
        std::cerr << "Could not write " << out << "\n";
        return 1;
    }
    std::cout << "Saved " << out << " (160x120)\nCAMERA TEST SUCCESS\n";
    return 0;
}
