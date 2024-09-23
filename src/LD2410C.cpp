#include "LD2410C.h"
#include <string.h>

// Constructor
LD2410C::LD2410C(HardwareSerial& serial) : _serial(serial), _configurationMode(false), _engineeringMode(false),
    _presenceDetected(false), _movingTargetDistance(0), _stationaryTargetDistance(0),
    _movingTargetEnergy(0), _stationaryTargetEnergy(0),
    _filteredMovingTargetDistance(0), _filteredStationaryTargetDistance(0),
    _filteredMovingTargetEnergy(0), _filteredStationaryTargetEnergy(0) {}

// Initialization
bool LD2410C::begin(unsigned long baud_rate) {
    _serial.begin(baud_rate);
    return true;
}

// Main update function
void LD2410C::update() {
    if (_serial.available()) {
        uint8_t frame[256];
        uint16_t frameLength = _serial.readBytes(frame, 256);
        parseFrame(frame, frameLength);
    }
}

// Frame parsing
void LD2410C::parseFrame(const uint8_t* frame, uint16_t length) {
    if (length >= 13 && frame[0] == 0xF4 && frame[1] == 0xF3 && frame[2] == 0xF2 && frame[3] == 0xF1) {
        uint8_t targetStatus = frame[6];
        _presenceDetected = (targetStatus != 0);
        
        float newMovingDistance = (frame[7] | (frame[8] << 8)) / 100.0f;
        uint8_t newMovingEnergy = frame[9];
        float newStationaryDistance = (frame[10] | (frame[11] << 8)) / 100.0f;
        uint8_t newStationaryEnergy = frame[12];

        uint32_t currentTime = getCurrentTime();

        // Apply adaptive filter
        _filteredMovingTargetDistance = _filter.filter(newMovingDistance, _filteredMovingTargetDistance, currentTime);
        _filteredStationaryTargetDistance = _filter.filter(newStationaryDistance, _filteredStationaryTargetDistance, currentTime);
        _filteredMovingTargetEnergy = _filter.filter(newMovingEnergy, _filteredMovingTargetEnergy, currentTime);
        _filteredStationaryTargetEnergy = _filter.filter(newStationaryEnergy, _filteredStationaryTargetEnergy, currentTime);

        // Call callbacks
        if (_presenceCallback) {
            _presenceCallback(_presenceDetected);
        }
        if (_distanceCallback) {
            _distanceCallback(_filteredMovingTargetDistance, _filteredStationaryTargetDistance);
        }
    }
}

// Adaptive filter methods
void LD2410C::enableAdaptiveFilter() {
    _filter.enable();
}

void LD2410C::disableAdaptiveFilter() {
    _filter.disable();
}

bool LD2410C::isAdaptiveFilterEnabled() const {
    return _filter.isEnabled();
}

// Helper methods for command sending and response reading
bool LD2410C::sendCommand(uint16_t command, const uint8_t* data, uint8_t dataLength) {
    uint8_t header[] = {0xFD, 0xFC, 0xFB, 0xFA};
    uint8_t footer[] = {0x04, 0x03, 0x02, 0x01};
    uint16_t length = dataLength + 2;

    _serial.write(header, sizeof(header));
    _serial.write(length & 0xFF);
    _serial.write((length >> 8) & 0xFF);
    _serial.write(command & 0xFF);
    _serial.write((command >> 8) & 0xFF);
    if (data && dataLength > 0) {
        _serial.write(data, dataLength);
    }
    _serial.write(footer, sizeof(footer));

    return true;
}

bool LD2410C::readResponse(uint8_t* response, uint16_t& responseLength, uint32_t timeout) {
    uint32_t startTime = getCurrentTime();
    uint16_t index = 0;
    bool frameStarted = false;

    while (getCurrentTime() - startTime < timeout) {
        if (_serial.available()) {
            uint8_t byte = _serial.read();
            if (!frameStarted && byte == 0xFD) {
                frameStarted = true;
                response[index++] = byte;
            } else if (frameStarted) {
                response[index++] = byte;
                if (index >= 4 && response[index-4] == 0x04 && response[index-3] == 0x03 && 
                    response[index-2] == 0x02 && response[index-1] == 0x01) {
                    responseLength = index;
                    return true;
                }
            }
            if (index >= 256) return false;
        }
    }
    return false;
}

uint32_t LD2410C::getCurrentTime() {
    return millis();
}
