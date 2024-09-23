#include "LD2410C.h"

// Constructor
LD2410C::LD2410C(HardwareSerial& serial) : _serial(serial), _configurationMode(false), _engineeringMode(false),
    _presenceDetected(false), _movingTargetDistance(0), _stationaryTargetDistance(0),
    _movingTargetEnergy(0), _stationaryTargetEnergy(0),
    _filteredMovingTargetDistance(0), _filteredStationaryTargetDistance(0),
    _filteredMovingTargetEnergy(0), _filteredStationaryTargetEnergy(0),
    _targetState(0), _detectionDistance(0), _maxMovementGate(0), _maxStationaryGate(0), _unoccupiedDuration(0) {
    memset(_movementEnergyGates, 0, sizeof(_movementEnergyGates));
    memset(_stationaryEnergyGates, 0, sizeof(_stationaryEnergyGates));
}
// Initialization
bool LD2410C::begin(unsigned long baud_rate) {
    _serial.begin(baud_rate);
    // TODO: Add any necessary initialization steps
    return true;
}

// Main update function
void LD2410C::update() {
    uint8_t frame[256];
    uint16_t frameLength;
    
    if (readResponse(frame, frameLength, 100)) {  // 100ms timeout
        parseFrame(frame, frameLength);
    }
}

// Configuration functions
bool LD2410C::enableConfiguration() {
    const uint8_t data[] = {0x01, 0x00};
    bool success = sendCommand(0x00FF, data, sizeof(data));
    if (success) {
        _configurationMode = true;
    }
    return success;
}

bool LD2410C::disableConfiguration() {
    bool success = sendCommand(0x00FE);
    if (success) {
        _configurationMode = false;
    }
    return success;
}

// Example of a configuration function
bool LD2410C::setMaxDetectionDistance(uint8_t movingTarget, uint8_t stationaryTarget) {
    if (!_configurationMode) return false;
    
    uint8_t data[] = {
        0x00, 0x00, movingTarget, 0x00, 0x00, 0x00,
        0x01, 0x00, stationaryTarget, 0x00, 0x00, 0x00
    };
    return sendCommand(0x0060, data, sizeof(data));
}

// Helper function to send commands
bool LD2410C::sendCommand(uint16_t command, const uint8_t* data, uint8_t dataLength) {
    uint8_t header[] = {0xFD, 0xFC, 0xFB, 0xFA};
    uint8_t footer[] = {0x04, 0x03, 0x02, 0x01};
    uint16_t length = dataLength + 2; // +2 for command

    _serial.write(header, sizeof(header));
    _serial.write(length & 0xFF);
    _serial.write((length >> 8) & 0xFF);
    _serial.write(command & 0xFF);
    _serial.write((command >> 8) & 0xFF);
    if (data && dataLength > 0) {
        _serial.write(data, dataLength);
    }
    _serial.write(footer, sizeof(footer));

    return true; // For simplicity, always return true. In a real implementation, you'd want to check for errors.
}


// Data parsing function
void LD2410C::parseFrame(const uint8_t* frame, uint16_t length) {
    if (length >= 13 && frame[0] == 0xF4 && frame[1] == 0xF3 && frame[2] == 0xF2 && frame[3] == 0xF1) {
        uint8_t dataType = frame[5];
        _targetState = frame[6];
        _presenceDetected = (_targetState != 0);
        
        float newMovingDistance = (frame[7] | (frame[8] << 8)) / 100.0f; // Convert to meters
        uint8_t newMovingEnergy = frame[9];
        float newStationaryDistance = (frame[10] | (frame[11] << 8)) / 100.0f; // Convert to meters
        uint8_t newStationaryEnergy = frame[12];
        _detectionDistance = (frame[13] | (frame[14] << 8)) / 100.0f; // Convert to meters

        uint32_t currentTime = getCurrentTime();

        // Apply adaptive filter
        _filteredMovingTargetDistance = _filter.filter(newMovingDistance, _filteredMovingTargetDistance, currentTime);
        _filteredStationaryTargetDistance = _filter.filter(newStationaryDistance, _filteredStationaryTargetDistance, currentTime);
        _filteredMovingTargetEnergy = _filter.filter(newMovingEnergy, _filteredMovingTargetEnergy, currentTime);
        _filteredStationaryTargetEnergy = _filter.filter(newStationaryEnergy, _filteredStationaryTargetEnergy, currentTime);

        // Update raw values
        _movingTargetDistance = newMovingDistance;
        _stationaryTargetDistance = newStationaryDistance;
        _movingTargetEnergy = newMovingEnergy;
        _stationaryTargetEnergy = newStationaryEnergy;

        // Parse engineering mode data if available
        if (dataType == 0x01 && length >= 32) {
            _maxMovementGate = frame[15];
            _maxStationaryGate = frame[16];
            for (int i = 0; i < 9; i++) {
                _movementEnergyGates[i] = frame[17 + i];
                _stationaryEnergyGates[i] = frame[26 + i];
            }
        }

        // Call callbacks if registered
        if (_presenceCallback) {
            _presenceCallback(_presenceDetected);
        }
        if (_distanceCallback) {
            _distanceCallback(_filteredMovingTargetDistance, _filteredStationaryTargetDistance);
        }
    }
}
// Data retrieval functions
bool LD2410C::isPresenceDetected() const {
    return _presenceDetected;
}
// Callback registration functions
void LD2410C::onPresenceDetected(std::function<void(bool)> callback) {
    _presenceCallback = callback;
}

void LD2410C::onDistanceChanged(std::function<void(float, float)> callback) {
    _distanceCallback = callback;
}

void LD2410C::enableAdaptiveFilter() {
    _filter.enable();
}

void LD2410C::disableAdaptiveFilter() {
    _filter.disable();
}

bool LD2410C::isAdaptiveFilterEnabled() const {
    return _filter.isEnabled();
}

// Update getter functions to return filtered values
float LD2410C::getMovingTargetDistance() const {
    return _filter.isEnabled() ? _filteredMovingTargetDistance : _movingTargetDistance;
}

float LD2410C::getStationaryTargetDistance() const {
    return _filter.isEnabled() ? _filteredStationaryTargetDistance : _stationaryTargetDistance;
}

uint8_t LD2410C::getMovingTargetEnergy() const {
    return _filter.isEnabled() ? _filteredMovingTargetEnergy : _movingTargetEnergy;
}

uint8_t LD2410C::getStationaryTargetEnergy() const {
    return _filter.isEnabled() ? _filteredStationaryTargetEnergy : _stationaryTargetEnergy;
}

// Implement getCurrentTime() based on your platform
// For example, if you're using Arduino:
uint32_t LD2410C::getCurrentTime() {
    return millis();
}

bool LD2410C::setSerialBaudRate(uint32_t baudRate) {
    if (!_configurationMode) return false;
    
    uint16_t baudRateIndex;
    switch (baudRate) {
        case 9600: baudRateIndex = 0x0001; break;
        case 19200: baudRateIndex = 0x0002; break;
        case 38400: baudRateIndex = 0x0003; break;
        case 57600: baudRateIndex = 0x0004; break;
        case 115200: baudRateIndex = 0x0005; break;
        case 230400: baudRateIndex = 0x0006; break;
        case 256000: baudRateIndex = 0x0007; break;
        case 460800: baudRateIndex = 0x0008; break;
        default: return false;  // Unsupported baud rate
    }
    
    uint8_t data[] = {baudRateIndex & 0xFF, (baudRateIndex >> 8) & 0xFF};
    return sendCommand(0x00A1, data, sizeof(data));
}

bool LD2410C::restoreFactorySettings() {
    if (!_configurationMode) return false;
    return sendCommand(0x00A2);
}

bool LD2410C::restartModule() {
    if (!_configurationMode) return false;
    return sendCommand(0x00A3);
}

bool LD2410C::setBluetoothState(bool enable) {
    if (!_configurationMode) return false;
    uint8_t data[] = {enable ? 0x01 : 0x00, 0x00};
    return sendCommand(0x00A4, data, sizeof(data));
}

bool LD2410C::getMacAddress(uint8_t* macAddress) {
    if (!_configurationMode || !macAddress) return false;
    
    uint8_t data[] = {0x01, 0x00};
    if (!sendCommand(0x00A5, data, sizeof(data))) return false;
    
    uint8_t response[256];
    uint16_t responseLength;
    if (!readResponse(response, responseLength)) return false;
    
    if (responseLength != 10 || response[0] != 0x00) return false;
    
    memcpy(macAddress, response + 1, 6);
    return true;
}

bool LD2410C::setBluetoothPassword(const char* password) {
    if (!_configurationMode || !password || strlen(password) != 6) return false;
    
    uint8_t data[6];
    for (int i = 0; i < 6; i++) {
        data[i] = password[i];
    }
    return sendCommand(0x00A9, data, sizeof(data));
}

bool LD2410C::setDistanceResolution(bool highResolution) {
    if (!_configurationMode) return false;
    uint8_t data[] = {highResolution ? 0x01 : 0x00, 0x00};
    return sendCommand(0x00AA, data, sizeof(data));
}

bool LD2410C::getDistanceResolution(bool& highResolution) {
    if (!_configurationMode) return false;
    
    if (!sendCommand(0x00AB)) return false;
    
    uint8_t response[256];
    uint16_t responseLength;
    if (!readResponse(response, responseLength)) return false;
    
    if (responseLength != 6 || response[0] != 0x00 || response[1] != 0x00) return false;
    
    highResolution = (response[4] == 0x01);
    return true;
}

bool LD2410C::getFirmwareVersion(uint16_t& major, uint16_t& minor) {
    if (!_configurationMode) return false;
    
    if (!sendCommand(0x00A0)) return false;
    
    uint8_t response[256];
    uint16_t responseLength;
    if (!readResponse(response, responseLength)) return false;
    
    if (responseLength != 12 || response[0] != 0x00 || response[1] != 0x00) return false;
    
    major = (response[6] << 8) | response[5];
    minor = (response[10] << 24) | (response[9] << 16) | (response[8] << 8) | response[7];
    return true;
}

bool LD2410C::readResponse(uint8_t* response, uint16_t& responseLength, uint32_t timeout) {
    uint32_t startTime = getCurrentTime();
    uint16_t index = 0;
    bool frameStarted = false;
    
    while (getCurrentTime() - startTime < timeout) {
        if (_serial.available()) {
            uint8_t byte = _serial.read();
            
            if (!frameStarted) {
                if (byte == 0xFD) {
                    frameStarted = true;
                    response[index++] = byte;
                }
            } else {
                response[index++] = byte;
                
                if (index >= 4 && response[index-4] == 0x04 && response[index-3] == 0x03 && 
                    response[index-2] == 0x02 && response[index-1] == 0x01) {
                    responseLength = index;
                    return true;
                }
            }
            
            if (index >= 256) {
                // Buffer overflow protection
                return false;
            }
        }
    }
    
    return false;  // Timeout
}

// Implement new data retrieval functions
uint8_t LD2410C::getTargetState() const {
    return _targetState;
}

float LD2410C::getDetectionDistance() const {
    return _detectionDistance;
}

uint8_t LD2410C::getMovementEnergyGate(uint8_t gate) const {
    if (gate < 9) {
        return _movementEnergyGates[gate];
    }
    return 0;
}

uint8_t LD2410C::getStationaryEnergyGate(uint8_t gate) const {
    if (gate < 9) {
        return _stationaryEnergyGates[gate];
    }
    return 0;
}

bool LD2410C::isEngineeringMode() const {
    return _engineeringMode;
}

uint8_t LD2410C::getMaxMovementGate() const {
    return _maxMovementGate;
}

uint8_t LD2410C::getMaxStationaryGate() const {
    return _maxStationaryGate;
}

uint16_t LD2410C::getUnoccupiedDuration() const {
    return _unoccupiedDuration;
}

bool LD2410C::readAllParameters() {
    if (!_configurationMode) return false;
    
    if (!sendCommand(0x0061)) return false;
    
    uint8_t response[256];
    uint16_t responseLength;
    if (!readResponse(response, responseLength)) return false;
    
    if (responseLength < 28 || response[0] != 0x00 || response[1] != 0x00) return false;
    
    _maxMovementGate = response[6];
    _maxStationaryGate = response[7];
    for (int i = 0; i < 9; i++) {
        _movementEnergyGates[i] = response[8 + i];
        _stationaryEnergyGates[i] = response[17 + i];
    }
    _unoccupiedDuration = (response[26] << 8) | response[27];
    
    return true;
}