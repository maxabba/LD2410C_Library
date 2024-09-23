#ifndef LD2410C_H
#define LD2410C_H

#include <Arduino.h>
#include <cstdint>
#include <functional>
#include "LD2410C_AdaptiveFilter.h"
#include <string.h>

class LD2410C {
public:
    // Constructor
    LD2410C(HardwareSerial& serial);

    // Initialization
    bool begin(unsigned long baud_rate = 256000);

    // Main update function, call this regularly
    void update();

    // Configuration functions
    bool enableConfiguration();
    bool disableConfiguration();
    bool setMaxDetectionDistance(uint8_t movingTarget, uint8_t stationaryTarget);
    bool setSensitivity(uint8_t gate, uint8_t movingSensitivity, uint8_t stationarySensitivity);
    bool setUnoccupiedDuration(uint16_t seconds);
    bool enableEngineeringMode();
    bool disableEngineeringMode();
    
    // New configuration functions
    bool setSerialBaudRate(uint32_t baudRate);
    bool restoreFactorySettings();
    bool restartModule();
    bool setBluetoothState(bool enable);
    bool getMacAddress(uint8_t* macAddress);
    bool setBluetoothPassword(const char* password);
    bool setDistanceResolution(bool highResolution);
    bool getDistanceResolution(bool& highResolution);

    // Data retrieval functions
    bool isPresenceDetected() const;
    float getMovingTargetDistance() const;
    float getStationaryTargetDistance() const;
    uint8_t getMovingTargetEnergy() const;
    uint8_t getStationaryTargetEnergy() const;
    bool getFirmwareVersion(uint16_t& major, uint16_t& minor);

    // New data retrieval functions
    uint8_t getTargetState() const;
    float getDetectionDistance() const;
    uint8_t getMovementEnergyGate(uint8_t gate) const;
    uint8_t getStationaryEnergyGate(uint8_t gate) const;
    bool isEngineeringMode() const;
    uint8_t getMaxMovementGate() const;
    uint8_t getMaxStationaryGate() const;
    uint16_t getUnoccupiedDuration() const;
    bool readAllParameters();

    // Callback registration
    void onPresenceDetected(std::function<void(bool)> callback);
    void onDistanceChanged(std::function<void(float, float)> callback);

    // Adaptive filter functions
    void enableAdaptiveFilter();
    void disableAdaptiveFilter();
    bool isAdaptiveFilterEnabled() const;

private:
    HardwareSerial& _serial;
    bool _configurationMode;
    bool _engineeringMode;
    
    // Sensor data
    bool _presenceDetected;
    float _movingTargetDistance;
    float _stationaryTargetDistance;
    uint8_t _movingTargetEnergy;
    uint8_t _stationaryTargetEnergy;

    // Filtered data
    float _filteredMovingTargetDistance;
    float _filteredStationaryTargetDistance;
    uint8_t _filteredMovingTargetEnergy;
    uint8_t _filteredStationaryTargetEnergy;

    // New private members for storing additional sensor data
    uint8_t _targetState;
    float _detectionDistance;
    uint8_t _movementEnergyGates[9];
    uint8_t _stationaryEnergyGates[9];
    uint8_t _maxMovementGate;
    uint8_t _maxStationaryGate;
    uint16_t _unoccupiedDuration;

    // Callbacks
    std::function<void(bool)> _presenceCallback;
    std::function<void(float, float)> _distanceCallback;

    // Adaptive filter
    AdaptiveFilter _filter;

    // Private helper functions
    bool sendCommand(uint16_t command, const uint8_t* data = nullptr, uint8_t dataLength = 0);
    bool readResponse(uint8_t* response, uint16_t& responseLength, uint32_t timeout = 1000);
    void parseFrame(const uint8_t* frame, uint16_t length);
    uint32_t getCurrentTime();
};

#endif // LD2410C_H