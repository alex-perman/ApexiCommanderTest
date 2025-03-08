#include "CanbusCommander.h"

void initPins() {
    pinMode(SCREEN_ON, OUTPUT);
    pinMode(UP_SW, INPUT);
    pinMode(DOWN_SW, INPUT);
    pinMode(LEFT_SW, INPUT);
    pinMode(RIGHT_SW, INPUT);
    pinMode(NEXT_SW, INPUT);
    pinMode(PREV_SW, INPUT);

    pinMode(CAN_TXD, INPUT);
    pinMode(CAN_RXD, OUTPUT);
}