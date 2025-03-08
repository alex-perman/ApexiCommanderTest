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
|   |   |   AirTemp         (Intake Air Temperature)    [degC]
|   |   |   BatVolt         (Battery Voltage)           [V]
ETC
|   Units
|   |   Metric
|   |   Imperial (cringe)
*/

#include <Arduino.h>
#include "CanbusCommander.h"
#include <U8g2lib.h>
#include <ESP32-TWAI-CAN.hpp>
#include "bitmaps.h"
#include "CAN_Display.h"
#include "CCfonts.h"

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

// Initialize the LCD for writing data to screen. Default 2 data display
int displayCount = 4;
CAN_Display LCD(displayCount);

// Buttons
int upPresses = 0, downPresses = 0, leftPresses = 0, rightPresses = 0, prevPresses = 0, nextPresses = 0;

// Menus
int menuPos[3] = {0, 0, 0};        // X, Y, PAGE {page0 = home, page1 = settings, page2 = etc, ...}


void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_lord_mr);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

bool getSW(int SW) {
    Serial.println("Button Pressed!");

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

void menuSelection(int menuNum) {
    int x, y, width, height;
    int xShift, yShift;
    switch (menuNum)
    {
    case 0:             // Main Menu
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
    case 1:         // Channel Select Menu
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
    case 2:         // Mode/ETC Select Menu
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

    menuSelection(0);

    u8g2.sendBuffer();

    delay(16); // ~60fps
}

void chanSelect() {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, screen_1_channel_select);

    menuSelection(1);

    u8g2.sendBuffer();

    delay(16); // ~60fps
}

void modeMenu() {
    u8g2.clearBuffer();
    u8g2.drawXBMP(0, 0, 128, 64, screen_2_etc_mode);

    menuSelection(2);

    u8g2.sendBuffer();

    delay(16); // ~60fps
}

void doMenus() {
    switch (menuPos[2])
    {
    case 0:
        mainMenu();
        break;
    case 1:
        chanSelect();
        break;
    case 2:
        modeMenu();
        break;
    default:
        break;
    }

    if (getSW(NEXT_SW)) {
        if (menuPos[2] == 0) {
            switch (menuPos[1])
            {
            case 0:
                menuPos[0] = 0;
                menuPos[1] = 0;
                menuPos[2] = 1;
                break;
            case 1:
                // Settings here
                break;
            case 2:
                menuPos[0] = 0;
                menuPos[1] = 0;
                menuPos[2] = 2;
            default:
                break;
            }
            while (getSW(NEXT_SW)) {
            }
        }
    }
    else if (getSW(PREV_SW)) {
        menuPos[0] = 0;
        menuPos[1] = 0;
        menuPos[2] = 0;
        while (getSW(PREV_SW)) {
        }
    }
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

        if(rxFrame.identifier == 0x7E8) {   // Standard OBD2 frame responce ID
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

void displayTest() {
    LCD.updateData(0, "Speed", 65.3);
    LCD.updateData(1, "RPM", 2300);
    LCD.updateData(2, "Temp", 90.5);
    LCD.updateData(3, "Fuel", 57.2);

    LCD.draw(u8g2);
    // displayCount = displayCount%4;
    // displayCount = 2^(displayCount);
    // LCD.setNumInputs(displayCount);
    delay(500);

}

void setup() {
    initPins();
    u8g2.begin();
    Serial.begin(115200);
    Serial.println("Serial monitor started!");
    u8g2_prepare();
    canSetup();                                 // Setup CANBUS
    digitalWrite(SCREEN_ON, HIGH);

    u8g2.setFont(u8g2_font_lord_mr);

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
    //doMenus();
    //canbusTest();
    displayTest();

}
