#ifndef EPUCK_HARDWARE_H
#define EPUCK_HARDWARE_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct EpuckSensors {
    std::array<uint16_t, 8> proximity{};
    bool valid = false;
};

class EpuckHardware {
public:
    explicit EpuckHardware(std::string ip, int port = 1000);

    bool connect();
    bool readSensors(EpuckSensors& sensors);
    bool sendRequest();
    bool receiveSensors(EpuckSensors& sensors);
    bool stop();
    bool setWheelSpeeds(int16_t left, int16_t right);

    // Camera: 160x120 RGB565 (38400 bytes). Saves as a BMP file.
    bool captureImage(std::vector<uint8_t>& rgb565);
    static bool saveImageBmp(const std::string& path,
                             const std::vector<uint8_t>& rgb565);
    void disconnect();

private:
    std::string ip_;
    int port_;
    int socket_ = -1;
    int16_t left_ = 0;
    int16_t right_ = 0;
};

#endif
