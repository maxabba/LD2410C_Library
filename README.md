# LD2410C Library

This library provides a comprehensive interface for the **LD2410C human presence radar sensor**. It enables the user to configure the sensor, retrieve real-time radar data, and apply adaptive filtering to smooth out noisy measurements. This library is designed for use in embedded systems, such as Arduino and ESP32, with PlatformIO as the preferred development environment.

## Features

- **Human Presence Detection**: Detects the presence of moving or stationary humans.
- **Adaptive Filtering**: Smoothes sensor readings to reduce noise and improve accuracy.
- **Serial Communication**: Communicates with the sensor via a UART interface.
- **Customizable Configuration**: Set maximum detection distances, sensitivity for different gates, and unoccupied duration.
- **Engineering Mode**: Access detailed data from the sensor in engineering mode.
- **Callback System**: Trigger custom functions when presence is detected or distances change.

## Library Contents

- **LD2410C.h**: The main header file that defines the class for interfacing with the sensor.
- **LD2410C.cpp**: The source file implementing the sensor communication and data processing.
- **LD2410C_AdaptiveFilter.h / LD2410C_AdaptiveFilter.cpp**: Implements an adaptive filter to smooth sensor readings.
- **Example Sketch**: A basic usage example (`BasicUsage.ino`) demonstrating how to use the library.

## Requirements

- **PlatformIO** (recommended) or **Arduino IDE**.
- A supported microcontroller (e.g., **ESP32**, **Arduino Uno**).
- **LD2410C radar sensor**.
  
## Installation

### Method 1: Install via PlatformIO (Recommended)

1. Open **VSCode** with the **PlatformIO** extension installed.
2. Open the **PlatformIO Home** tab, and search for `LD2410C` in the **Library Manager**.
3. Click **Install** to add the library to your project.

### Method 2: Install from GitHub

1. Clone the repository from GitHub:
   ```bash
   git clone https://github.com/yourusername/LD2410C_Library.git

Copy the contents of the LD2410C_Library directory into your project’s lib/ folder.
Method 3: Manual Installation (Arduino IDE)
Download the ZIP file from the repository.
In Arduino IDE, go to Sketch > Include Library > Add .ZIP Library, and select the downloaded ZIP file.
Usage
Here is a simple example demonstrating how to use the LD2410C library:

Example: BasicUsage.ino
cpp
Copia codice
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
Explanation
Initialization: The sensor is initialized using ld2410c.begin().
Callbacks: Register callbacks to respond to presence detection and distance changes using onPresenceDetected() and onDistanceChanged().
Main Loop: The ld2410c.update() function must be called frequently to handle sensor data reception.
API Reference
Initialization
bool begin(unsigned long baud_rate = 256000)
Initializes the LD2410C sensor with the specified baud rate. Returns true on success.
Configuration Functions
bool enableConfiguration()
Enables configuration mode, allowing you to send configuration commands.

bool disableConfiguration()
Disables configuration mode and resumes normal operation.

bool setMaxDetectionDistance(uint8_t movingTarget, uint8_t stationaryTarget)
Configures the maximum detection distance for moving and stationary targets.

bool setSensitivity(uint8_t gate, uint8_t movingSensitivity, uint8_t stationarySensitivity)
Sets the sensitivity for the specified distance gate.

bool enableEngineeringMode()
Enables engineering mode, which provides additional raw sensor data.

Data Retrieval Functions
bool isPresenceDetected() const
Returns true if the sensor detects human presence.

float getMovingTargetDistance() const
Retrieves the distance to the nearest moving target.

float getStationaryTargetDistance() const
Retrieves the distance to the nearest stationary target.

Adaptive Filter Functions
void enableAdaptiveFilter()
Enables the adaptive filter for smoothing data.

void disableAdaptiveFilter()
Disables the adaptive filter.

bool isAdaptiveFilterEnabled() const
Returns true if the adaptive filter is enabled.

Callback Registration
void onPresenceDetected(std::function<void(bool)> callback)
Registers a callback function for when presence detection changes.

void onDistanceChanged(std::function<void(float, float)> callback)
Registers a callback function for distance changes (moving and stationary targets).

Development
If you wish to modify the library or contribute to its development:

Clone the repository:
bash
Copia codice
git clone https://github.com/yourusername/LD2410C_Library.git
Make your changes in the source files under the src/ directory.
Commit your changes and push them to GitHub:
bash
Copia codice
git add .
git commit -m "Your commit message"
git push
Testing
The library can be tested using PlatformIO’s built-in tools:

Connect your microcontroller.
Build and upload the example by clicking the PlatformIO: Build and Upload buttons.
Open the serial monitor to observe the output:
bash
Copia codice
platformio device monitor
Contributing
We welcome contributions! Feel free to open a pull request or report issues if you find bugs or have suggestions for improvements.

License
This project is licensed under the MIT License. See the LICENSE file for details.

markdown
Copia codice

### **Explanation of the README.md:**

1. **Title and Overview:** 
   - Clearly explains what the library is, its purpose, and the features it provides.
   
2. **Features:** 
   - Highlights key features of the library, such as presence detection, adaptive filtering, and serial communication.

3. **Installation Instructions:** 
   - Provides multiple ways to install the library, including PlatformIO (recommended), GitHub, and manual installation for Arduino IDE.

4. **Basic Usage Example:** 
   - A simple example that demonstrates initializing the sensor, registering callbacks, and reading sensor data.

5. **API Reference:** 
   - Explains key functions provided by the library for initialization, configuration, data retrieval, and adaptive filtering.

6. **Development and Contribution:** 
   - Provides instructions on how to contribute to the library, clone the repository, and submit changes.

7. **License:** 
   - Specifies that the project is licensed under the MIT License.

### **Next Steps:**

1. **Add this `README.md` to your project root.**
2. **Push the changes to GitHub** by following the Git instructions from earlier:
   ```bash
   git add README.md
   git commit -m "Add README"
   git push
Ensure that all example code and instructions are tested and working correctly before publishing the library.