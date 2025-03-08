#ifndef CAN_DISPLAY_H
#define CAN_DISPLAY_H

#include <Arduino.h>
#include <U8g2lib.h>  // Include your LCD library

class CAN_Display {
private:
    int numInputs;
    static const int MAX_INPUTS = 8;

    struct DataEntry {
        String label;
        float value;
    };

    DataEntry entries[MAX_INPUTS];

public:
    CAN_Display(int numInputs);
    void setNumInputs(int newNumInputs);
    void updateData(int index, String label, float value);
    void draw(U8G2 &display);
};

#endif
