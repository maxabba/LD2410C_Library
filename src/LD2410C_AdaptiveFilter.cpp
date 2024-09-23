#include "LD2410C_AdaptiveFilter.h"

AdaptiveFilter::AdaptiveFilter() : _enabled(false), _lastUpdateTime(0), _alpha(0.5f) {
}

void AdaptiveFilter::enable() {
    _enabled = true;
}

void AdaptiveFilter::disable() {
    _enabled = false;
}

bool AdaptiveFilter::isEnabled() const {
    return _enabled;
}

float AdaptiveFilter::filter(float newValue, float previousValue, uint32_t currentTime) {
    if (!_enabled) return newValue;

    // Time-dependent filtering can be added if needed, this is a simple implementation
    return previousValue + _alpha * (newValue - previousValue);
}

uint8_t AdaptiveFilter::filter(uint8_t newValue, uint8_t previousValue, uint32_t currentTime) {
    if (!_enabled) return newValue;

    // Applying the same alpha smoothing to uint8 values, casting to float for calculation
    return static_cast<uint8_t>(previousValue + _alpha * (newValue - previousValue));
}
