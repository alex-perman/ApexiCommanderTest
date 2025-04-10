#pragma once
#include <Arduino.h>
#include "driver/twai.h"

#define MAX_CHANNELS 8

class CANDataManager {
public:
    void begin();                       // Initializes internal state
    void update();                      // Polls CAN bus (non-blocking)
    float getData(int channel);        // Returns latest cached value
    bool isDataFresh(int channel);     // True if updated in last 1000ms
    void setCustomID(int channel, uint32_t id);

private:
    float dataCache[MAX_CHANNELS];
    unsigned long lastUpdate[MAX_CHANNELS];
    uint32_t customCANID[MAX_CHANNELS];
};
