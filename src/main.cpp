/*
CANBUS COMMANDER TESTING
ALEX PERMAN 2025
*/

/*
MAIN MENU
|   Channel Select
|   |   Data Display
SETTING
|   Channel Select
|   |   Data Select
|   |   |   Inj Duty Cycle  (Injector Duty Cycle)       [%]
|   |   |   IgnT Ld         (Leading Ignition Timing)   [deg]
|   |   |   IgnT Tr         (Trailing Ignition Timing)  [deg]
|   |   |   Eng Rev         (Engine RPM)                [RPM]
|   |   |   Speed           (Vehicle Speed)             [km/h]
|   |   |   Boost           (Intake MAP)                [bar]
|   |   |   Knock           (Detonation Level)          [!!!]
|   |   |   WtrTemp         (Water Temperature)         [degC]
|   |   |   OilTemp         (Oil Temperature)           [degC]
|   |   |   OilPres         (Oil Pressure)              [bar]
|   |   |   AirTemp         (Intake Air Temperature)    [degC]
|   |   |   BatVolt         (Battery Voltage)           [V]
|   |   |   Lambda          (Air Fuel Ratio Lambda)     []
ETC
|   Units
|   |   Metric
|   |   Imperial (cringe)
*/

#include <Arduino.h>
#include "CanbusCommander.h"
#include <U8g2lib.h>
#include <ESP32-TWAI-CAN.hpp>
#include "driver/twai.h"  // Native ESP32 CAN driver
#include "bitmaps.h"
#include "CANDataManager.h"
#include "CCfonts.h"
#include <Preferences.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define MAX_DROPLETS 5  // Number of spill droplets

const bool BOOTSCREEN = true;

// CAN Setup
CanFrame rxFrame;
CANDataManager canManager;

// Preferences
Preferences preferences;

// Cup position variables
float cupX = 64;
float accelTarget = 0.8;
float velocity = 0;
float acceleration = 0;
float damping = 0.85;

// Liquid animation variables
float liquidOffset = 0;  // The tilt of the liquid surface
float liquidVelocity = 0;
float liquidDamping = 0.95;  // Higher damping for smoother motion

// Spill effect
struct Droplet {
    float x, y, vy;
    bool active;
} droplets[MAX_DROPLETS];

// Function to initialize a new droplet
void spawnDroplet(float startX, float startY) {
    for (int i = 0; i < MAX_DROPLETS; i++) {
        if (!droplets[i].active) {
            droplets[i] = {startX, startY, (float)random(1, 3) / 2.0f, true};  // Random speed
            break;
        }
    }
}

int mod(int dividend, int divisor) {
    return ((dividend % divisor) + divisor) % divisor;
}

// Screen Setup
//U8G2_KS0108_128X64_F u8g2(U8G2_R0, 8, 9, 10, 11, 4, 5, 6, 7, /*enable=*/ 18, /*dc=*/ 17, /*cs0=*/ 14, /*cs1=*/ 15, /*cs2=*/ U8X8_PIN_NONE, /* reset=*/  U8X8_PIN_NONE); 	// Set R/W to low!
//U8G2_KS0108_128X64_F u8g2(U8G2_R0, 21, 17, 16, 19, 18, 5, 4, 23, /*enable=*/ 26, /*dc=*/ 25, /*cs0=*/ 22, /*cs1=*/ 14, /*cs2=*/ U8X8_PIN_NONE, /* reset=*/  U8X8_PIN_NONE);   // Set R/W to low!
U8G2_KS0108_128X64_F u8g2(U8G2_R0, 4, 5, 6, 7, 15, 16, 17, 18, /*enable=*/ 10, /*dc=*/ 9, /*cs0=*/ 3, /*cs1=*/ 46, /*cs2=*/ U8X8_PIN_NONE, /* reset=*/  U8X8_PIN_NONE);   // Set R/W to low!

// Buttons
int upPresses = 0, downPresses = 0, leftPresses = 0, rightPresses = 0, prevPresses = 0, nextPresses = 0;

// Menus
int menuPos[3] = {0, 0, 0};         // X, Y, PAGE {page0 = home, page1 = settings, page2 = etc, ...}
const char * paramList[8] = {"Knock", "Boost", "Eng Rev", "Speed", "Oil Temp", "Wtr Temp", "Air Temp", "BatVolt"};      // Array of parameters!
uint8_t customCANID[12] =   {   0x00,    0x00,      0x00,    0x00,       0x00,       0x00,       0x00,      0x00};      // Stores *CUSTOM* CANBUS ID of all parameters as set by user
int selectedCANID[8];               // Stores indicies of customCANID[] that are selected by user to be displayed. Index 0 is dataNum1, up to index 7 is dataNum8
int paramCursor = 8;                // set up to start at zero and count to 7 for each parameter selected.
int paramLocation[8][2];

/***************** PREFERENCES *********************/
void saveCANIDS() {
    preferences.begin("myApp", false);
    preferences.putBytes("customCANIDs", customCANID, sizeof(customCANID));
    preferences.putBytes("selectedCANIDs", selectedCANID, sizeof(selectedCANID));
    preferences.putBytes("paramLocation", paramLocation, sizeof(paramLocation));
    preferences.end();
}

void loadCANIDS() {
    preferences.begin("myApp", true);
    size_t customBytes = preferences.getBytes("customCANIDs", customCANID, sizeof(customCANID));
    size_t selectedBytes = preferences.getBytes("selectedCANIDs", selectedCANID, sizeof(selectedCANID));
    size_t locationBytes = preferences.getBytes("paramLocation", paramLocation, sizeof(paramLocation));
    
    if (customBytes != sizeof(customCANID)) {
        memset(customCANID, 0, sizeof(customCANID));
    }
    if (selectedBytes != sizeof(selectedCANID)) {
        memset(selectedCANID, 0, sizeof(selectedCANID));
    }
    if (locationBytes != sizeof(paramLocation)) {
        memset(paramLocation, 0, sizeof(paramLocation));
    }

    preferences.end();
}
/***************************************************/

void u8g2_prepare(void) {
  //u8g2.setFont(u8g2_font_lord_mr);
  u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

/*******************************************
                CANBUS CODE 
 *******************************************/

void canSetup() {
  // Set pins
  ESP32Can.setPins(CAN_TXD, CAN_RXD);

  // You can set custom size for the queues - those are default
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);

  // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
  // but you can easily convert it from numerical value using .convertSpeed()
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

  // You can also just use .begin()..
  if(ESP32Can.begin()) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }

  // or override everything in one command;
  // It is also safe to use .begin() without .end() as it calls it internally
  if(ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TXD, CAN_RXD, 10, 10)) {
      Serial.println("CAN bus started!");
  } else {
      Serial.println("CAN bus failed!");
  }
}

// float getData(int param) {
//     twai_message_t message;

//     // Non-blocking call — returns immediately
//     if (twai_receive(&message, 0) == ESP_OK) {
//         if (message.identifier == customCANID[param]) {
//             switch (param)
//             {
//             case 1: // Boost (MAP)
//                 return message.data[2];
//             case 2: // RPM
//                 return (256 * message.data[2] + message.data[3]) / 4.0;
//             case 3: // Speed
//                 return message.data[2];
//             case 4: case 5: case 6: // Temps
//                 return message.data[2] - 40;
//             case 7: // Voltage
//                 return message.data[2];
//             default:
//                 return -100;
//             }
//         }
//     }

//     // No frame or no match
//     return -100;
// }

float getDataOLD(int param) {
    if (ESP32Can.readFrame(rxFrame)) {          // 1000ms timeout removed
        if (rxFrame.identifier == customCANID[param]) {
            switch (param)
            {
            case 1:                 // Boost (MAP) [kPa]
                return rxFrame.data[2];
                break;
            case 2:                 // ENG REV [rpm]
                return (256*rxFrame.data[2]+rxFrame.data[3])/4;
                break;
            case 3:                 // Vehicle Speed [km/h]
                return rxFrame.data[2];
                break;
            case 4:                 // Oil Temp [ºC]
                return rxFrame.data[2] - 40;
                break;
            case 5:                 // Water Temp [ºC]
                return rxFrame.data[2] - 40;
                break;
            case 6:                 // IAT [ºC]
                return rxFrame.data[2] - 40;
                break;
            case 7:                 // Battery Voltage [V]
                return rxFrame.data[2];
                break;

            default:
                break;
            }   
        }

    }
    else {
        return -100;    // READ CAN ERROR, DISPLAY ---
    }
}

void canbusTest() {
    
    // You can set custom timeout, default is 1000
    if(ESP32Can.readFrame(rxFrame, 1000)) {
        // Comment out if too many requests 
        Serial.printf("%03X,", rxFrame.identifier);
        Serial.printf("%X,", rxFrame.rtr);
        Serial.printf("%X,", rxFrame.data_length_code);
        //Serial.printf("%X", rxFrame.data);

        for (int i = 0; i < rxFrame.data_length_code; i++) {
          Serial.printf("%X",rxFrame.data[i]);
        }
        Serial.printf("\r\n");
        
        // Serial.printf("%03X ", rxFrame.identifier);
        // if (rxFrame.rtr) { Serial.printf("%X", rxFrame.data); }

        if(rxFrame.identifier == 0x7E8) {   // Standard OBD2 frame response ID
            Serial.printf("Coolant temp: %3d°C \r\n", rxFrame.data[3] - 40); // Convert to °C
        }

        if(rxFrame.identifier == 0x180) {       // ENGINE RPM
          //int RPM = (rxFrame.data[3]*256 + rxFrame.data[4])/10;
        //   int TPS = rxFrame.data[2];
            u8g2.clearBuffer();

            char buffer[10];
            if (rxFrame.data[0] == 11) {
                sprintf(buffer, "%d", rxFrame.data[2]);
                u8g2.drawStr(0, 10, buffer);
            }
            if (rxFrame.data[0] == 10) {
                sprintf(buffer, "%d", rxFrame.data[1]);
                u8g2.drawStr(0, 20, buffer);
            }
            // if (rxFrame.data[0] == 11) {
            //     sprintf(buffer, "%d", (float)rxFrame.data[1]);
            //     u8g2.drawStr(0, 20, buffer);
            // }

        }
        // if (rxFrame.identifier == 0x02) {
        //     char buffer[10];
        //     if (rxFrame.data[0] == 11) {
        //         sprintf(buffer, "%d", rxFrame.data[1]);
        //         u8g2.drawStr(0, 20, buffer);
        //     }
        // }
        else {
            u8g2.drawStr(0, 20, "NO DATA");
        }
    }
    //u8g2.drawStr(0, 20, "NO DATA");
    u8g2.sendBuffer();
}

/************************************************************/

bool getSW(int SW) {
    //Serial.println("Button Pressed!");

    switch (SW)
    {
    case UP_SW:
        if (digitalRead(UP_SW)) return 1; else return 0;
        break;
    case DOWN_SW:
        if (digitalRead(DOWN_SW)) return 1; else return 0;
        break;
    case LEFT_SW:
        if (digitalRead(LEFT_SW)) return 1; else return 0;
        break;
    case RIGHT_SW:
        if (digitalRead(RIGHT_SW)) return 1; else return 0;
        break;
    case PREV_SW:
        if (digitalRead(PREV_SW)) return 1; else return 0;
        break;
    case NEXT_SW:
        if (digitalRead(NEXT_SW)) return 1; else return 0;
        break;
    default:
        return 0;
        break;
    }
}

void buttonTest() {
    u8g2.clearBuffer();

// Menu Alignment Lines
//  display.drawLine(0,21,128,21, SSD1306_WHITE);
//  display.drawLine(0,43,128,43, SSD1306_WHITE);
//  display.drawLine(63,0,63,63, SSD1306_WHITE);
//  display.drawLine(65,0,65,63, SSD1306_WHITE);
    u8g2_prepare();
    //u8g2.setFont(u8g2_font_luRS18_tf);
    // u8g2.drawStr(23, 5, "monitor");
    // u8g2.drawStr(23, 25, "setting");
    // u8g2.drawStr(48,45, "etc.");

    char buffer[10];
    sprintf(buffer, "   UP: %d", (int)getSW(UP_SW));
    u8g2.drawStr(0, 0, buffer);;

    sprintf(buffer, " DOWN: %d", (int)getSW(DOWN_SW));
    u8g2.drawStr(0, 10, buffer);

    sprintf(buffer, " LEFT: %d", (int)getSW(LEFT_SW));
    u8g2.drawStr(0, 20, buffer);

    sprintf(buffer, "RIGHT: %d", (int)getSW(RIGHT_SW));
    u8g2.drawStr(0, 30, buffer);

    sprintf(buffer, " PREV: %d", (int)getSW(PREV_SW));
    u8g2.drawStr(0, 40, buffer);

    sprintf(buffer, " NEXT: %d", (int)getSW(NEXT_SW));
    u8g2.drawStr(0, 50, buffer);

    // bitmap
    u8g2.drawXBMP(64, 0, 32, 64, cupBitmap);

    u8g2.sendBuffer();
}

void cupTest() {
    // Read accelerometer (pseudo-code)
    if (digitalRead(RIGHT_SW) && (acceleration < accelTarget)) {
        acceleration += 0.09;
    }
    else if (digitalRead(LEFT_SW) && (acceleration > -accelTarget)) {
        acceleration -= 0.09;
    }
    else {
        acceleration = 0;
    }
    Serial.print("Acc = ");
    Serial.println(acceleration);

    // Update cup movement physics
    //acceleration = accX * 2.0;
    velocity += acceleration;
    velocity *= damping;
    cupX += velocity;

    // Boundaries
    if (cupX < 0) { cupX = 0; velocity = 0; acceleration = 0; }
    if (cupX > 128-32) { cupX = 128-32; velocity = 0; acceleration = 0; }

    // Update liquid movement (sloshing effect)
    float targetOffset = acceleration * 4;  // More acceleration = more tilt
    liquidVelocity += (targetOffset - liquidOffset) * 0.25;  // Smooth transition
    liquidVelocity *= liquidDamping;
    liquidOffset += liquidVelocity;

    // Check for spill
    if (abs(liquidOffset) > 6) {  // Threshold for spilling
        spawnDroplet(cupX + (liquidOffset > 0 ? 14 : 2), 42);  // Spawn droplet at spill edge
    }

    // Update droplets
    for (int i = 0; i < MAX_DROPLETS; i++) {
        if (droplets[i].active) {
            droplets[i].y += droplets[i].vy;  // Apply gravity
            if (droplets[i].y > 64) droplets[i].active = false;  // Remove if off-screen
        }
    }

    // Draw frame
    u8g2.clearBuffer();
    
    // Draw cup bitmap
    u8g2.drawXBMP((int)cupX, 0, 32, 64, cupBitmap);

    // Draw liquid as a wavy line
    for (int i = 0; i < 30; i++) {
        int waveY = 30 + (i-15)*liquidOffset*0.2 + (sin(i * liquidOffset));  // Wavy effect
        u8g2.drawPixel(cupX + 1 + i, waveY);
    }

    // Draw spilled droplets
    for (int i = 0; i < MAX_DROPLETS; i++) {
        if (droplets[i].active) {
            u8g2.drawPixel((int)droplets[i].x, (int)droplets[i].y);
        }
    }

    u8g2.sendBuffer();
    delay(16);  // ~60 FPS
}

/************************* MENU SELECTION **************************/

void menuSelection(int menuNum) {
    int x, y, width, height;
    int xShift, yShift;
    switch (menuNum)
    {
    case 00:             // Main Menu
        x = 15;
        y = 6;
        width = 99;
        height = 15;
        yShift = 19;
        xShift = 40;

        if (getSW(UP_SW)) {
            menuPos[1] = mod(menuPos[1] - 1, 3);
            while (getSW(UP_SW)) {
            }
        }
        if (getSW(DOWN_SW)) {
            menuPos[1] = mod(menuPos[1] + 1, 3);
            while (getSW(DOWN_SW)) {
            }
        }
        break;
    case 10:         // Channel Select Menu
        x = 30;
        y = 12;
        width = 70;
        height = 9;
        yShift = 10;
        xShift = 40;

        if (getSW(UP_SW)) {
            menuPos[1] = mod(menuPos[1] - 1, 4);
            while (getSW(UP_SW)) {
            }
        }
        if (getSW(DOWN_SW)) {
            menuPos[1] = mod(menuPos[1] + 1, 4);
            while (getSW(DOWN_SW)) {
            }
        }
        break;
    case 20:         // CAN ID Config Menu
        x = 3;
        y = 7;
        width = 60;
        height = 9;
        xShift = 62;
        yShift = 10;

        if (getSW(UP_SW)) {
            menuPos[1] = mod(menuPos[1] - 1, 4);
            while (getSW(UP_SW)) {
            }
            if(menuPos[1] == 3) {
                menuPos[0] = mod(menuPos[0] - 1, 2);
            }
        }
        if (getSW(DOWN_SW)) {
            menuPos[1] = mod(menuPos[1] + 1, 4);
            while (getSW(DOWN_SW)) {
            }
            if(menuPos[1] == 0) {
                menuPos[0] = mod(menuPos[0] + 1, 2);
            }
        }
        
        if (getSW(LEFT_SW)) {               // this shit needs to REMEMBER LOL ya glhf gn EDIT FIXED LFGGG
            if (paramCursor == 8) { 
                paramCursor = 0; 
                memset(selectedCANID, -1, sizeof(selectedCANID));    // clear array
            }
            size_t index = menuPos[1] + 4 * menuPos[0];
            if (selectedCANID[paramCursor] == -1) {
                selectedCANID[paramCursor] = index;
                paramLocation[paramCursor][0] = menuPos[0];         // Store X location
                paramLocation[paramCursor][1] = menuPos[1];         // Store Y location
                paramCursor++;
            }

            while (getSW(LEFT_SW)) {
            }
        }

        char buffer[5];
        u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);
        for (int i=0; i<paramCursor; i++) {
            sprintf(buffer, "%d", i+1);
            u8g2.drawStr((x + 3) + paramLocation[i][0] * xShift, y + paramLocation[i][1] * yShift, buffer); 
        }
        break;
    case 30:         // Mode/ETC Select Menu
        x = 20;
        y = 8;
        width = 90;
        height = 9;
        yShift = 10;
        xShift = 40;

        if (getSW(UP_SW)) {
            menuPos[1] = mod(menuPos[1] - 1, 3);
            while (getSW(UP_SW)) {
            }
        }
        if (getSW(DOWN_SW)) {
            menuPos[1] = mod(menuPos[1] + 1, 3);
            while (getSW(DOWN_SW)) {
            }
        }
        break;
    
    default:
        break;
    }

    // if DOWN add ySHIFT to y
    // if RIGHT add xSHIFT to x
    u8g2.setDrawColor(2);
    u8g2.drawBox(x + menuPos[0] * xShift, y + menuPos[1] * yShift, width, height);
}

void mainMenu() {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, screen_0_main_menu);

    menuSelection(00);

    u8g2.sendBuffer();

    delay(16); // ~60fps
}

void chanSelect() {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, screen_1_channel_select);

    menuSelection(10);

    u8g2.sendBuffer();

    delay(16); // ~60fps
}

void canID_config() {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, screen_3_data_select);

    menuSelection(20);

    if (getSW(LEFT_SW)) {

    }

    u8g2.sendBuffer();

    delay(16); // ~60fps
}

void setCANID() {
    u8g2.clearBuffer();

    char buffer[50];
    // Add bounds checking
    size_t index = menuPos[1] + 4 * menuPos[0];
    if (index < 8) {  // Assuming paramList has 8 elements
        u8g2.drawStr(4, 4, "Set CANBUS ID for:");
        snprintf(buffer, sizeof(buffer), "%s", paramList[index]);
        u8g2.drawStr(4, 13, buffer);
    } else {
        // Handle error - index out of bounds
        u8g2.drawStr(9, 10, "Invalid Selection");
    }

    u8g2.setFont(u8g2_font_ncenB14_tr);
    sprintf(buffer, "0x%02X", customCANID[index]);
    u8g2.drawStr(64 - u8g2.getStrWidth(buffer)/2, 32, buffer);
    u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);

    int digit;
    if (getSW(LEFT_SW)) { digit = 1; }
    else if (getSW(RIGHT_SW)) { digit = 0; }
    else { digit = -1; }

    if (digit != -1) {
        u8g2.setDrawColor(2);
        u8g2.drawBox(64 - u8g2.getStrWidth(buffer)/2 + 14*digit, 57, 15, 2);        // am tired fix this later

        if (getSW(UP_SW)) {
            customCANID[index] = customCANID[index] + 0x01 << (digit*4);
            while(getSW(UP_SW)) {
            }
        }
        if (getSW(DOWN_SW)) {
            customCANID[index] = customCANID[index] - 0x01 << (digit*4);
            while(getSW(DOWN_SW)) {
            }
        }
    }

    u8g2.sendBuffer();
    delay(16);
}

void modeMenu() {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, screen_2_etc_mode);

    menuSelection(30);

    u8g2.sendBuffer();

    delay(16); // ~60fps
}

void dispUnits(int x, int y, int idPos) {       // idPos from 0 to 8 to match selectedCANID[]
    switch (selectedCANID[idPos])
    {
    case 0:                 // Knock
        u8g2.drawStr(x, y, "!!!");
        break;
    case 1:                 // BOOST
        u8g2.drawStr(x, y, "bar");
        break;
    case 2:                 // ENG RPM
        u8g2.drawStr(x, y, "rpm");
        break;
    case 3:                 // SPEED
        u8g2.drawStr(x, y, "km/h");
        break;
    case 4:                 // Oil Temp
        u8g2.drawStr(x, y, "deg");
        break;
    case 5:                 // Water Temp
        u8g2.drawStr(x, y, "deg");
        break;
    case 6:                 // Air Temp
        u8g2.drawStr(x, y, "deg");
        break;
    case 7:                 // Batt Volt
        u8g2.drawStr(x, y, "V");
        break;
    default:
        break;
    }
}

void chan_1() {         // Display the data at customCANID[0]
    u8g2.clearBuffer();
    char buffer[50];
    // Read CANBUS here
    canManager.update();        // Put this outside of 16ms delay if losing frames....
    
    //u8g2.setFont(u8g2_font_ncenB14_tr);         // need even bigger text!
    u8g2.setFont(u8g2_font_timB24_tn);
    //u8g2.drawStr(32, 18, "3581");
    float data = canManager.getData(selectedCANID[0]);
    if (data == -100) {
        u8g2.drawStr(32, 18, "---");
    }
    else {
        //sprintf(buffer, "%.2f", data);
        switch (selectedCANID[0]) {
                case 0: case 2: case 3:  // No DP for knock, RPM, Speed
                    sprintf(buffer, "%.0f", data);
                    break;
                case 1: case 4: case 5: case 6:  // 1 DP for boost, temps
                    sprintf(buffer, "%.1f", data);
                    break;
                case 7:  // 2 DP for battV
                    sprintf(buffer, "%.2f", data);
                    break;
                default:    // Default 1dp
                    sprintf(buffer, "%.1f", data);
                    break;
            }
        //u8g2.drawStr(32, 18, buffer);
        u8g2.drawStr(108 - u8g2.getStrWidth(buffer), 18, buffer);
    }

    u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);
    sprintf(buffer, "%s, 0x%02X", paramList[selectedCANID[0]], customCANID[selectedCANID[0]]);
    u8g2.drawStr(1, 1, buffer);

    dispUnits(96, 56, 0);
    
    u8g2.sendBuffer();
    delay(16);
}

void chan_2() {         // Display the data at customCANID[0] and [1]
    u8g2.clearBuffer();
    char buffer[50];

    // This can probably be a for loop?????

    // Read Canbus Here
    canManager.update();

    // Both lol
    for (int i = 0; i < 2; i++) {
        u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);
        sprintf(buffer, "%s, 0x%02X", paramList[selectedCANID[i]], customCANID[selectedCANID[i]]);
        u8g2.drawStr(1, 1 + 32*i, buffer);

        u8g2.setFont(u8g2_font_ncenB14_tr);         // 14 pt??
        int x = 70;                                 // ez right align
        float data = canManager.getData(selectedCANID[i]);          // ??? COMMENT FOR UNDERSTANDING!!!
        if (data == -100) {
            u8g2.drawStr(x - u8g2.getStrWidth("---"), 10 + 32*i, "---");
        }
        else {
            //sprintf(buffer, "%.2f", data);
            switch (selectedCANID[i]) {
                case 0: case 2: case 3:  // No DP for knock, RPM, Speed
                    sprintf(buffer, "%.0f", data);
                    break;
                case 1: case 4: case 5: case 6:  // 1 DP for boost, temps
                    sprintf(buffer, "%.1f", data);
                    break;
                case 7:  // 2 DP for battV
                    sprintf(buffer, "%.2f", data);
                    break;
                default:    // Default 1dp
                    sprintf(buffer, "%.1f", data);
                    break;
            }

            u8g2.drawStr(x - u8g2.getStrWidth(buffer), 10 + 32*i, buffer);
        }

        dispUnits(x + 10, 10 + 32*i, i);
    }

    u8g2.sendBuffer();
    delay(16);
}

void chan_4() {         // Display the data at customCANID[0..3]
    u8g2.clearBuffer();
    char buffer[50];

    // Read Canbus Here
    canManager.update();

    // Both lol
    for (int i = 0; i < 4; i++) {
        u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);
        //sprintf(buffer, "%s, 0x%02X", paramList[selectedCANID[i]], customCANID[selectedCANID[i]]);
        sprintf(buffer, "%s", paramList[selectedCANID[i]]);         // no display canid ~:^( 
        u8g2.drawStr(45 - u8g2.getStrWidth(buffer), 3 + 16*i, buffer);

        u8g2.setFont(u8g2_font_bytesize_tr);         // 12 pt??
        int x = 95;                                 // ez right align
        float data = canManager.getData(selectedCANID[i]);
        if (data == -100) {
            u8g2.drawStr(x - u8g2.getStrWidth("---"), 1 + 16*i, "---");
        }
        else {
            //sprintf(buffer, "%.1f", data);
            switch (selectedCANID[i]) {
                case 0: case 2: case 3:  // No DP for knock, RPM, Speed
                    sprintf(buffer, "%.0f", data);
                    break;
                case 1: case 4: case 5: case 6:  // 1 DP for boost, temps
                    sprintf(buffer, "%.1f", data);
                    break;
                case 7:  // 2 DP for battV
                    sprintf(buffer, "%.2f", data);
                    break;
                default:    // Default 1dp
                    sprintf(buffer, "%.1f", data);
                    break;
            }
            u8g2.drawStr(x - u8g2.getStrWidth(buffer), 1 + 16*i, buffer);
        }

        dispUnits(x + 3, 1 + 16*i, i);
    }

    u8g2.sendBuffer();
    delay(16);
}

void chan_8() {         // Display the data at customCANID[0..7]
    u8g2.clearBuffer();
    char buffer[50];

    // Read Canbus Here
    canManager.update();

    // Both lol
    for (int i = 0; i < 8; i++) {
        u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);
        //sprintf(buffer, "%s, 0x%02X", paramList[selectedCANID[i]], customCANID[selectedCANID[i]]);
        sprintf(buffer, "%s", paramList[selectedCANID[i]]);         // no display canid ~:^( 
        u8g2.drawStr(50 - u8g2.getStrWidth(buffer), 8*i, buffer);

        // Same Font
        int x = 100;                                 // ez right align
        float data = canManager.getData(selectedCANID[i]);
        if (data == -100) {
            u8g2.drawStr(x - u8g2.getStrWidth("---"), 8*i, "---");
        }
        else {
            // sprintf(buffer, "%.1f", data);
            switch (selectedCANID[i]) {
                case 0: case 2: case 3:  // No DP for knock, RPM, Speed
                    sprintf(buffer, "%.0f", data);
                    break;
                case 1: case 4: case 5: case 6:  // 1 DP for boost, temps
                    sprintf(buffer, "%.1f", data);
                    break;
                case 7:  // 2 DP for battV
                    sprintf(buffer, "%.2f", data);
                    break;
                default:    // Default 1dp
                    sprintf(buffer, "%.1f", data);
                    break;
            }
            u8g2.drawStr(x - u8g2.getStrWidth(buffer), 8*i, buffer);
        }

        dispUnits(x + 3, 8*i, i);
    }

    u8g2.sendBuffer();
    delay(16);
}

void doMenus() {
    switch (menuPos[2])
    {
    case 00:
        mainMenu();
        break;
    case 10:
        chanSelect();
        break;
    case 11:
        chan_1();
        break;
    case 12:
        chan_2();
        break;
    case 13:
        chan_4();
        break;
    case 14:
        chan_8();
        break;
    case 20:
        canID_config();
        break;
    case 21:
        setCANID();
        break;
    case 30:
        modeMenu();
        break;
    default:
        break;
    }

    if (getSW(RIGHT_SW)) {
        if (menuPos[2] == 20) {
            menuPos[2] = 21;
        }
    }

    if (getSW(NEXT_SW)) {
        if (menuPos[2] == 00) {
            switch (menuPos[1])
            {
            case 0:
                menuPos[0] = 0;
                menuPos[1] = 0;
                menuPos[2] = 10;
                break;
            case 1:
                // Settings here
                menuPos[0] = 0;
                menuPos[1] = 0;
                menuPos[2] = 20;
                loadCANIDS();
                paramCursor = 8;
                break;
            case 2:
                menuPos[0] = 0;
                menuPos[1] = 0;
                menuPos[2] = 30;
                break;
            default:
                break;
            }
            while (getSW(NEXT_SW)) {
            }
        }
        else if (menuPos[2] == 10) {
            switch (menuPos[1])
            {
            case 0:                 // 1 Channel
                // menuPos[0] = 0;
                // menuPos[1] = 0;
                menuPos[2] = 11;
                break;
            case 1:                 // 2 Channel
                // Settings here
                // menuPos[0] = 0;
                // menuPos[1] = 0;
                menuPos[2] = 12;
                loadCANIDS();
                paramCursor = 8;
                break;
            case 2:                 // 4 Channel
                // menuPos[0] = 0;
                // menuPos[1] = 0;
                menuPos[2] = 13;
                break;
            case 3:                 // 8 Channel
                // menuPos[0] = 0;
                // menuPos[1] = 0;
                menuPos[2] = 14;
                break;
            default:
                break;
            }
            while (getSW(NEXT_SW)) {
            }
        }
        else if (menuPos[2] == 20) {
            u8g2.clearBuffer();
            if (paramCursor == 8) {
                saveCANIDS();
                for (int i = 0; i < sizeof(customCANID); i++) {
                    canManager.setCustomID(i, customCANID[i]);       // Load CANIDs into canManager
                }
                //u8g2.setDrawColor(0);
                //u8g2.drawBox(32, 16, 64, 32);
                u8g2.setFont(u8g2_font_ncenB14_tr);
                u8g2.drawStr(37, 25, "Saved!");
                u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);
                u8g2.drawStr(49, 21, "CAN IDs");
                u8g2.sendBuffer();
                delay(500);

                menuPos[0] = 0;
                menuPos[1] = 0;
                menuPos[2] = 00;    // GOTO main menu
            }
            else {
                // DONT saveCANIDS
                u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);
                u8g2.drawStr(5, 21, "Please Select 8");
                u8g2.drawStr(15, 29, "Parameters!");
                u8g2.sendBuffer();
                delay(1500);

                menuPos[2] = 20;    // GOTO main menu
            }

            while (getSW(NEXT_SW)) {
            }
        }
    }
    else if (getSW(PREV_SW)) {
        if (menuPos[2] == 21) {
            menuPos[2] = 20;
        }
        else if (menuPos[2] == 11 || menuPos[2] == 12 || menuPos[2] == 13 || menuPos[2] == 14) {
            menuPos[2] = 10;
        }
        else {
            menuPos[0] = 0;
            menuPos[1] = 0;
            menuPos[2] = 00;
        }
        while (getSW(PREV_SW)) {
        }
    }
}

void setup() {
    initPins();
    digitalWrite(SCREEN_ON, LOW);
    u8g2.begin();
    Serial.begin(115200);
    while (!Serial) { delay(10); }              // Remove this after debugging!
    Serial.println("Serial monitor started!");
    u8g2_prepare();
    canSetup();                                 // Setup CANBUS
    loadCANIDS();                               // Load CANIDs into memory from flash

    canManager.begin();
    for (int i = 0; i < sizeof(customCANID); i++) {
        canManager.setCustomID(i, customCANID[i]);       // Load CANIDs into canManager
    }

    digitalWrite(SCREEN_ON, HIGH);

    //u8g2.setFont(u8g2_font_lord_mr);
    u8g2.setFont(u8g2_font_pfc_sans_v1_1_tf);

    if (BOOTSCREEN) {
        u8g2.clearBuffer();
        u8g2.drawXBMP(0, 0, 128, 64, boot_logo);
        u8g2.sendBuffer();
        delay(2000);
    }
    

    for (int i = 0; i < MAX_DROPLETS; i++) droplets[i].active = false;  // Reset droplets
}

void loop() {
    //buttonTest();
    //cupTest();
    doMenus();
    //canbusTest();
    //displayTest();
    //canID_config();
}
