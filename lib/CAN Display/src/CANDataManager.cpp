#include "CANDataManager.h"

void CANDataManager::begin() {
    for (int i = 0; i < MAX_CHANNELS; i++) {
        dataCache[i] = -100;
        lastUpdate[i] = 0;
        customCANID[i] = 0;
    }
}

void CANDataManager::setCustomID(int channel, uint32_t id) {
    if (channel >= 0 && channel < MAX_CHANNELS) {
        customCANID[channel] = id;
    }
}

void CANDataManager::update() {
    twai_message_t message;

    while (twai_receive(&message, 0) == ESP_OK) {
        for (int i = 0; i < MAX_CHANNELS; i++) {
            if (message.identifier == customCANID[i]) {
            //if (message.data[0] == customCANID[i]) {
                switch (i) {
                    case 0:
                        dataCache[i] = message.data[0];
                        break;
                    case 1:
                        dataCache[i] = (256 * message.data[0] + message.data[1]) / 4.0;
                        break;
                    case 2:
                        dataCache[i] = message.data[0];
                        break;
                    case 3: case 4: case 5:
                        dataCache[i] = message.data[0] - 40;
                        break;
                    case 6:
                        dataCache[i] = message.data[0];
                        break;
                    default:
                        dataCache[i] = 8008;
                        break;
                }
                lastUpdate[i] = millis();
            }
        }
    }
}

float CANDataManager::getData(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return -100;

    if (millis() - lastUpdate[channel] > 1000) {
        return -100; // stale
    }

    return dataCache[channel];
}

bool CANDataManager::isDataFresh(int channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return false;
    return millis() - lastUpdate[channel] <= 1000;
}
