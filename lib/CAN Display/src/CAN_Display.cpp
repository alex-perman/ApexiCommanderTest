#include "CAN_Display.h"

CAN_Display::CAN_Display(int numInputs) {
    this->numInputs = (numInputs >= 1 && numInputs <= MAX_INPUTS) ? numInputs : MAX_INPUTS;
}

void CAN_Display::setNumInputs(int newNumInputs) {
    if (newNumInputs >= 1 && newNumInputs <= MAX_INPUTS) {
        numInputs = newNumInputs;
    }
}

void CAN_Display::updateData(int index, String label, float value) {
    if (index >= 0 && index < numInputs) {
        entries[index].label = label;
        entries[index].value = value;
    }
}

void CAN_Display::draw(U8G2 &display) {
    display.clearBuffer();
    int spacing = 64 / numInputs;
    for (int i = 0; i < numInputs; i++) {
        display.setCursor(0, (i + 1) * spacing);
        display.print(entries[i].label + ": " + String(entries[i].value));
    }
    display.sendBuffer();
}
