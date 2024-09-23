#include <Arduino.h>
#include "LD2410C.h"

// Set up the sensor on hardware serial
HardwareSerial sensorSerial(2); // (Pin configuration depends on your board)
LD2410C ld2410c(sensorSerial);

void setup() {
    Serial.begin(115200);
    sensorSerial.begin(256000);
    
    // Initialize the sensor
    if (ld2410c.begin()) {
        Serial.println("LD2410C initialized successfully!");
    } else {
        Serial.println("Failed to initialize LD2410C!");
    }

    // Set up callbacks
    ld2410c.onPresenceDetected([](bool detected) {
        Serial.print("Presence detected: ");
        Serial.println(detected);
    });

    ld2410c.onDistanceChanged([](float movingDistance, float stationaryDistance) {
        Serial.print("Moving Target Distance: ");
        Serial.println(movingDistance);
        Serial.print("Stationary Target Distance: ");
        Serial.println(stationaryDistance);
    });
}

void loop() {
    ld2410c.update(); // Call regularly to handle sensor data
}
