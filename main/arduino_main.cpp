/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

// Bare minimum code for spinning motors triggered by controller input

// Assumes servo is connected to pin 15

// Assumes motor controller IN1 and IN2 are connected to pins 14 and 12

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <bits/stdc++.h>
#include <Arduino_APDS9960.h>


#define IN1 12
#define IN2 14
#define BIN1 27
#define BIN2 26

//Color Sensing
#define APDS9960_INT 0
#define I2C_SDA 21
#define I2C_SLC 22
#define I2C_FREQ 100000

Servo servo;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

TwoWire I2C_0 = TwoWire(0);
APDS9960 sensor = APDS9960(I2C_0, APDS9960_INT);

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}


void setup() {
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

 
    //servo.attach(15);

    // motor controller outputs
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(2, OUTPUT);

    //Color sensor attachments
    I2C_0.begin(I2C_SDA, I2C_SLC, I2C_FREQ);
    sensor.setInterruptPin(APDS9960_INT);
    sensor.begin();
    
   
    Serial.begin(115200);
}

void loop() {
    // BP32.update();

    while(!sensor.colorAvailable()) {
        delay(5);
    }

    int r, g, b, a;
    sensor.readColor(r, g, b, a);

    // Serial.print("r = ");
    // Serial.print(r);
    // Serial.print("g = ");
    // Serial.print(g);
    // Serial.print("b = ");
    // Serial.print(b);

    if (r > 100) {
        Serial.print("RED is DETECTED");
    } else if (g > 100) {
        Serial.print("GREEN is DETECTED");
    } else if (b > 100) {
        Serial.print("BLUE is DETECTED");
    } else {
        Serial.print("detecting....");
    }


    // Serial.println("Looping");
    // digitalWrite(IN1, HIGH);
    // digitalWrite(IN2, LOW);
    // digitalWrite(BIN1, LOW);
    // digitalWrite(BIN2, HIGH);

    //     delay(2000);

    // Serial.println(" DC motor stop");
    //     digitalWrite(IN1, LOW);
    //     digitalWrite(IN2, LOW);
    //     digitalWrite(BIN1, LOW);
    //     digitalWrite(BIN2, LOW);



    // Controller Code
    // for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    //     GamepadPtr controller = myGamepads[i];
    //     if (controller && controller->isConnected()) {
            
    //         Serial.println("Connected");
    //         digitalWrite(2, HIGH);
    //         delay(2000);
    //         digitalWrite(2, LOW);
           
    //         // if (controller->l1() == 1) {
    //         //     Serial.print("Servo move");
    //         //     servo.write(1000);
    //         // }
    //         // if (controller->l1() == 0) {
    //         //     Serial.print("Servo stop");
    //         //     servo.write(1500);
    //         // }

    //         //Move forward
    //         if(controller->axisY() < 0) { // negative y is upward on stick
    //             Serial.println(" DC motor move");
    //             digitalWrite(IN1, HIGH);
    //             digitalWrite(IN2, LOW);
    //             digitalWrite(BIN1, LOW);
    //             digitalWrite(BIN2, HIGH);
    //         }

    //         //Move backward
    //         if(controller->axisY() > 0) {
    //             Serial.println(" DC motor move");
    //             digitalWrite(IN1, LOW);
    //             digitalWrite(IN2, HIGH);
    //             digitalWrite(BIN1, HIGH);
    //             digitalWrite(BIN2, LOW);
    //         }

    //         //Stop moving
    //         if(controller->axisY() == 0) { // negative y is upward on stick
    //             Serial.println(" DC motor move");
    //             digitalWrite(IN1, LOW);
    //             digitalWrite(IN2, LOW);
    //             digitalWrite(BIN1, LOW);
    //             digitalWrite(BIN2, LOW);
    //         }

    //         // if(controller->axisRX() > 0) { 
    //         //     Serial.println(" DC motor move");
    //         //     digitalWrite(IN1, LOW);
    //         //     digitalWrite(IN2, HIGH);
    //         // }
    //         // if(controller->axisRX() < 0) { // stop motor 1
    //         //     Serial.println(" DC motor stop");
    //         //     digitalWrite(IN1, LOW);
    //         //     digitalWrite(IN2, LOW);
    //         // }

    //         // if(controller->axisRX() = 0) { // stop motor 1
    //         //     Serial.println(" DC motor stop");
    //         //     digitalWrite(IN1, LOW);
    //         //     digitalWrite(IN2, LOW);
    //         }

    //         // PHYSICAL BUTTON A
    //         if (controller->b()) {
    //             Serial.println("button a pressed");
    //         }

    // }
    vTaskDelay(1);
}
