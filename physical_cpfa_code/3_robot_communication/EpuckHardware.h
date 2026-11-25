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
    // Plays a sound with the next packet. Bits as in the e-puck2 firmware
    // (Asercom2.c): 0x01 Mario, 0x02 Underworld, 0x04 Star Wars,
    // 0x08 short beep, 0x10 long beep, 0x20 stop, 0x40 We Wish You.
    void playSound(uint8_t soundBits);
    // Green body LED ring; stays as set until changed (stop() turns it off).
    void setBodyLed(bool on);
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
    uint8_t sound_ = 0;
    uint8_t leds_ = 0;  // packet byte 7: 0x10 = body LED
    unsigned long extraPackets_ = 0;  // replies skipped to stay on the newest one
};

#endif
