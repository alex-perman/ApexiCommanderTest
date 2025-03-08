#ifndef MY_BOARD_H
#define MY_BOARD_H

#include <Arduino.h>

// Example pin assignments
const uint8_t SCREEN_ON = 8;
const uint8_t UP_SW = 12;
const uint8_t DOWN_SW = 11;
const uint8_t LEFT_SW = 48;
const uint8_t RIGHT_SW = 13;
const uint8_t NEXT_SW = 14;
const uint8_t PREV_SW = 1;


// More pin mappings...
const uint8_t CAN_TXD = 21;
const uint8_t CAN_RXD = 47;

// Function prototypes (optional)
void initPins();

#endif // MY_BOARD_H
