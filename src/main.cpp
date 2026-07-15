// Firmware shell scaffold. Real behaviour arrives with tasks/T04 (serial
// control + LEDC servo output); until then this only proves the toolchain.

#include <Arduino.h>

#include "arm_core/version.h"

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("{\"type\":\"hello\",\"fw\":\"%s\",\"proto\":%d,\"profile\":\"scaffold\"}\n",
                  ARM_FW_VERSION, ARM_PROTO_VERSION);
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last >= 5000) {
        last = millis();
        Serial.println("[scaffold] alive — next: tasks/T01");
    }
}
