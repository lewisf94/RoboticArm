// Firmware shell: wires arm_core (portable logic) to arm_hal (LEDC servo
// output) over USB serial. Boots disabled/detached; motion requires an
// explicit `enable` command (docs/architecture.md safety model). WiFi/WS
// (T06/T07), NVS trim persistence and the e-stop pin (T05) arrive later.

#include <Arduino.h>

#include "arm_core/motion.h"
#include "arm_core/profiles/bench_3dof.h"
#include "arm_core/protocol.h"
#include "arm_core/version.h"
#include "arm_hal/ledc_servo_output.h"
#include "pins.h"

namespace {

constexpr uint32_t kTickIntervalMs = 20;  // 50 Hz, matches the servo PWM frame

const arm::ArmProfile& profile = arm::kBench3Dof;

arm::MotionController motion(profile);
arm_hal::LedcServoOutput servo_output(pins::kServoGpio, profile.n_joints);

bool enabled = false;

void on_enable(bool on, void* /*ctx*/) {
    if (on) {
        for (uint8_t j = 0; j < profile.n_joints; ++j) {
            servo_output.attach(profile.joints[j].channel);
        }
    } else {
        servo_output.detach_all();
    }
}

void on_estop(void* /*ctx*/) {
    servo_output.detach_all();
}

bool persist_trim_stub(uint8_t /*j*/, float /*deg*/, void* /*ctx*/) {
    return true;  // real NVS persistence arrives in T05
}

arm::Protocol protocol(motion, profile,
                        arm::Protocol::SystemHooks{&enabled, on_enable, on_estop, persist_trim_stub, nullptr});

char line_buf[512];
size_t line_len = 0;
bool line_overflowed = false;

uint32_t last_tick_ms = 0;

void send_reply(const char* line) {
    char out[512];
    const size_t n = protocol.handle_line(line, out, sizeof(out));
    Serial.write(reinterpret_cast<const uint8_t*>(out), n);
    Serial.write('\n');
}

// Non-blocking: drains whatever the UART/USB-CDC buffer currently holds and
// returns. A line longer than line_buf is discarded (reported as bad_json
// once its newline arrives) rather than silently truncated or dropped mid-
// buffer, so a runaway line can never be misread as a shorter, valid one.
void poll_serial() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (c == '\n') {
            if (line_overflowed) {
                send_reply("");  // -> err bad_json
                line_overflowed = false;
            } else {
                line_buf[line_len] = '\0';
                send_reply(line_buf);
            }
            line_len = 0;
            continue;
        }
        if (c == '\r') continue;
        if (line_overflowed) continue;

        if (line_len + 1 >= sizeof(line_buf)) {
            line_overflowed = true;
            line_len = 0;
            continue;
        }
        line_buf[line_len++] = c;
    }
}

void tick_motion(uint32_t dt_ms) {
    motion.tick(dt_ms);
    if (!enabled) return;
    for (uint8_t j = 0; j < profile.n_joints; ++j) {
        servo_output.write_us(profile.joints[j].channel, motion.joint(j).output_us());
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("{\"type\":\"hello\",\"fw\":\"%s\",\"proto\":%d,\"profile\":\"%s\",\"joints\":%d}\n",
                  ARM_FW_VERSION, ARM_PROTO_VERSION, profile.name, profile.n_joints);
    last_tick_ms = millis();
}

void loop() {
    poll_serial();

    const uint32_t now = millis();
    const uint32_t dt = now - last_tick_ms;
    if (dt >= kTickIntervalMs) {
        last_tick_ms = now;
        tick_motion(dt);
    }
}
