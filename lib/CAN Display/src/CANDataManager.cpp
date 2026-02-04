#include "CANDataManager.h"

void CANDataManager::begin() {
    // Serial.begin(115200);
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

void CANDataManager::setOBDPID(int channel, uint8_t pid) {
    if (channel >= 0 && channel < MAX_CHANNELS) {
        obdPID[channel] = pid;
    }
}

void CANDataManager::requestOBD2Data() {
    static unsigned long lastRequest = 0;
    unsigned long now = millis();
    
    // // Request PIDs every 5ms to avoid flooding the bus
    // if (now - lastRequest < 5) {
    //     return;
    // }
    // lastRequest = now;
    
    // Request one channel at a time (cycling through)
    static int currentChannel = 0;
    
    // Skip channels with no PID set
    int attempts = 0;
    while (obdPID[currentChannel] == 0 && attempts < MAX_CHANNELS) {
        currentChannel = (currentChannel + 1) % MAX_CHANNELS;
        attempts++;
    }
    
    // If no PIDs are set, don't send request
    if (obdPID[currentChannel] == 0) {
        return;
    }
    
    twai_message_t request;
    request.identifier = 0x7DF;  // OBD2 broadcast ID
    request.extd = 0;            // Standard frame
    request.rtr = 0;             // Data frame
    request.data_length_code = 8;
    
    // OBD2 Mode 01 (current data) request format
    request.data[0] = 0x02;                     // Number of additional bytes
    request.data[1] = 0x01;                     // Mode 01 - Show current data
    request.data[2] = obdPID[currentChannel];   // PID
    request.data[3] = 0x00;
    request.data[4] = 0x00;
    request.data[5] = 0x00;
    request.data[6] = 0x00;
    request.data[7] = 0x00;
    
    twai_transmit(&request, pdMS_TO_TICKS(10));
    
    // Move to next channel for next request
    currentChannel = (currentChannel + 1) % MAX_CHANNELS;
}

void CANDataManager::parseOBD2Response(twai_message_t &message) {
    // OBD2 response format:
    // Byte 0: Number of additional bytes
    // Byte 1: Mode + 0x40 (0x41 for mode 01 response)
    // Byte 2: PID
    // Byte 3+: Data
    
    if (message.data_length_code < 3) return;
    
    uint8_t mode = message.data[1];
    uint8_t pid = message.data[2];
    
    // Check if it's a Mode 01 response
    if (mode != 0x41) return;
    
    // Find which channel this PID corresponds to
    for (int channel = 0; channel < MAX_CHANNELS; channel++) {
        if (obdPID[channel] == pid) {
            float value = -100;
            
            // Decode based on PID
            switch (pid) {
                case 0x0C:  // Engine RPM
                    value = ((message.data[3] * 256.0) + message.data[4]) / 4.0;
                    break;
                    
                case 0x0D:  // Vehicle Speed
                    value = message.data[3];
                    break;
                case 0x11:  // Throttle Position
                    value = message.data[3]*100.0/255.0;
                    break;
                case 0x05:  // Engine Coolant Temperature
                case 0x0F:  // Intake Air Temperature
                case 0x46:  // Ambient Air Temperature
                case 0x5C:  // Engine Oil Temperature
                    value = message.data[3] - 40;
                    break;
                    
                case 0x04:  // Calculated Engine Load
                    value = message.data[3] * 100.0 / 255.0;  // Percentage
                    break;
                    
                case 0x0B:  // Intake Manifold Pressure
                    value = message.data[3] / 100;  // bar
                    break;
                    
                case 0x42:  // Battery Voltage (if supported)
                    value = ((message.data[3] * 256.0) + message.data[4]) / 1000.0;
                    break;
                    
                default:
                    // Generic handling for unknown PIDs
                    value = message.data[3];
                    break;
            }
            
            dataCache[channel] = value;
            lastUpdate[channel] = millis();
            break;  // Found the channel, no need to continue
        }
    }
}

void CANDataManager::update() {
    twai_message_t message;

    if (useOBD2 == false) {                             // USE CUSTOM IDS
        while (twai_receive(&message, 0) == ESP_OK) {
            for (int i = 0; i < MAX_CHANNELS; i++) {
                if (message.identifier == customCANID[i]) {
                //if (message.data[0] == customCANID[i]) {
                    switch (i) {
                        case 0:
                            dataCache[i] = message.data[0];
                            break;
                        case 1:
                            dataCache[i] = message.data[0];
                            break;
                        case 2:
                            dataCache[i] = (256 * message.data[0] + message.data[1]) / 4.0;
                            break;
                        case 3: case 4: case 5:
                            dataCache[i] = message.data[0] - 40;
                            break;
                        case 6:
                            dataCache[i] = message.data[0];
                            break;
                        case 7:
                            dataCache[i] = (256 * message.data[0] + message.data[1]) / 100.0;
                            break;
                        default:
                            dataCache[i] = 8008;
                            break;
                    }
                    lastUpdate[i] = millis();
                    // Serial.print("dataCache[%d] = %f", i, dataCache[i]);
                }
            }
        }
    }
    else if (useOBD2) {                         // USE OBD2 IDs
        // Request OBD2 PIDs for each channel
        requestOBD2Data();
        
        // Process responses
        while (twai_receive(&message, 0) == ESP_OK) {
            // Check if it's an OBD2 response (0x7E8 - 0x7EF)
            if (message.identifier >= 0x7E8 && message.identifier <= 0x7EF) {
                parseOBD2Response(message);
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
