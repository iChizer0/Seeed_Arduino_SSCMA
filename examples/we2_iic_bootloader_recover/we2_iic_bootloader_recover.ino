#include <Arduino.h>
#include <Wire.h>

#include "firmware.h"

void wireInit();

size_t scanWire();

int ep2_isp_i2c_proc(uint8_t* file_buf, uint32_t file_size);

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(false);
    Serial.println();

    wireInit();

    size_t avail = 0;

Scan:
    avail = scanWire();
    if (avail == 0) {
        Serial.println("No I2C devices found");
        delay(1000);
        goto Scan;
    }

    Serial.println("Press 'enter' to start recover WE2 bootloader (or 'q' to restart)");
}

void (*reboot)() = 0;

void loop() {
    // press 'enter' to continue
    int c = Serial.read();
    while (Serial.read() != -1)
        ;

    switch (c) {
    case 'Q':
    case 'q':
        reboot();

        break;

    case '\r':
    case '\n':
        Serial.println("Recovering WE2 bootloader...");
        ep2_isp_i2c_proc((uint8_t*)firmware, firmware_len);
        Serial.println("Done");
        break;
    default:
        break;
    }

    delay(10);
}