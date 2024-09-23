#ifndef LD2410C_ADAPTIVEFILTER_H
#define LD2410C_ADAPTIVEFILTER_H

#include <cstdint>

class AdaptiveFilter {
public:
    AdaptiveFilter();

    // Enable and disable the filter
    void enable();
    void disable();
    bool isEnabled() const;

    // Filter function for different data types
    float filter(float newValue, float previousValue, uint32_t currentTime);
    uint8_t filter(uint8_t newValue, uint8_t previousValue, uint32_t currentTime);

private:
    bool _enabled;
    uint32_t _lastUpdateTime;
    float _alpha; // Smoothing factor, adjust this based on system responsiveness
};

#endif // LD2410C_ADAPTIVEFILTER_H
