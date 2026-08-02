// Firmware shell: wires arm_core (portable logic) to arm_hal (LEDC servo
// output) over USB serial. Boots disabled/detached; motion requires an
// explicit `enable` command (docs/architecture.md safety model).
// T05: trims persisted in NVS, physical e-stop input, RGB status LED,
// 10 Hz serial telemetry via the `stream` command.
// T06: WiFi (STA with AP fallback, mDNS) and `wifi_set`.
// T07: HTTP + WebSocket transport (web_server), serving web/ from LittleFS.

#include <Arduino.h>
#include <Preferences.h>

#include "arm_core/motion.h"
#include "arm_core/profiles/bench_3dof.h"
#include "arm_core/protocol.h"
#include "arm_core/version.h"
#include "arm_hal/ledc_servo_output.h"
#include "pins.h"
#include "web_server.h"
#include "wifi_manager.h"

namespace {

constexpr uint32_t kTickIntervalMs = 20;    // 50 Hz, matches the servo PWM frame
constexpr uint32_t kStreamIntervalMs = 100;  // 10 Hz telemetry
constexpr uint32_t kDebounceMs = 20;
constexpr uint32_t kLedBlinkMs = 250;

const arm::ArmProfile& profile = arm::kBench3Dof;

arm::MotionController motion(profile);
arm_hal::LedcServoOutput servo_output(pins::kServoGpio, profile.n_joints);
Preferences prefs;

bool enabled = false;
bool estop_pin_active = false;  // debounced state of the physical e-stop input

void trim_key(uint8_t j, char* buf, size_t n) {
    snprintf(buf, n, "trim%u", j);
}

void load_trims() {
    if (!prefs.begin("arm", /*readOnly=*/true)) return;  // namespace absent on first boot
    for (uint8_t j = 0; j < profile.n_joints; ++j) {
        char key[8];
        trim_key(j, key, sizeof(key));
        motion.joint(j).set_trim_deg(prefs.getFloat(key, 0.0f));
    }
    prefs.end();
}

bool persist_trim(uint8_t j, float deg, void* /*ctx*/) {
    char key[8];
    trim_key(j, key, sizeof(key));
    if (!prefs.begin("arm", /*readOnly=*/false)) return false;
    const size_t written = prefs.putFloat(key, deg);
    prefs.end();
    return written == sizeof(float);
}

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

uint32_t free_heap(void* /*ctx*/) {
    return ESP.getFreeHeap();
}

bool inhibit_enable(void* /*ctx*/) {
    return estop_pin_active;
}

bool reboot_pending = false;
uint32_t reboot_at_ms = 0;

// Never calls ESP.restart() directly: the ack for wifi_set must reach the
// client over serial before the device disappears, and that write happens
// in send_reply() after handle_line() (and this hook) returns. poll_reboot()
// does the actual restart once the delay has elapsed.
void request_reboot(uint32_t delay_ms, void* /*ctx*/) {
    reboot_pending = true;
    reboot_at_ms = millis() + delay_ms;
}

bool set_wifi_creds(const char* ssid, const char* pass, void* /*ctx*/) {
    return wifi_manager::save_creds(ssid, pass);
}

arm::Protocol::WifiInfo wifi_info(void* /*ctx*/) {
    const wifi_manager::Status s = wifi_manager::status();

    const char* mode_str = "off";
    switch (s.mode) {
        case wifi_manager::Mode::kConnectingSta: mode_str = "connecting"; break;
        case wifi_manager::Mode::kSta: mode_str = "sta"; break;
        case wifi_manager::Mode::kAp: mode_str = "ap"; break;
        case wifi_manager::Mode::kOff: mode_str = "off"; break;
    }

    // static: outlives this call, which is all Protocol needs (it copies the
    // string into the JSON document immediately - see protocol.cpp) but a
    // stack buffer would not.
    static char ip_buf[16];
    snprintf(ip_buf, sizeof(ip_buf), "%u.%u.%u.%u", s.ip[0], s.ip[1], s.ip[2], s.ip[3]);

    return arm::Protocol::WifiInfo{mode_str, ip_buf, s.rssi};
}

arm::Protocol protocol(motion, profile,
                        arm::Protocol::SystemHooks{&enabled, on_enable, on_estop, persist_trim, nullptr,
                                                    free_heap, inhibit_enable, set_wifi_creds, request_reboot,
                                                    wifi_info});

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

// E-stop input: normally-closed switch to GND with the internal pull-up, so
// closed (safe) reads LOW and open (triggered - or not wired at all!) reads
// HIGH. Fail-safe on purpose: a bench without a switch must jumper GPIO 10
// to GND or `enable` is refused (docs/hardware.md). Opening the switch acts
// exactly like the estop command; closing it again clears the inhibit but
// the arm stays disabled until an explicit `enable`.
bool estop_raw_prev = false;
uint32_t estop_edge_ms = 0;

void poll_estop(uint32_t now) {
    const bool raw = digitalRead(pins::kEstop) == HIGH;
    if (raw != estop_raw_prev) {
        estop_raw_prev = raw;
        estop_edge_ms = now;
        return;
    }
    if (now - estop_edge_ms < kDebounceMs) return;

    if (raw && !estop_pin_active) {
        estop_pin_active = true;
        enabled = false;
        servo_output.detach_all();
    } else if (!raw && estop_pin_active) {
        estop_pin_active = false;
    }
}

// Status LED (WS2812 on GPIO 48): red = disabled, green = enabled,
// blue blink = e-stop latched. Kept dim; neopixelWrite is cheap enough to
// call at loop rate but only rewritten when the derived color changes.
void update_led(uint32_t now) {
    static uint8_t last_r = 255, last_g = 255, last_b = 255;
    uint8_t r = 0, g = 0, b = 0;
    if (estop_pin_active) {
        b = ((now / kLedBlinkMs) & 1u) ? 48 : 0;
    } else if (enabled) {
        g = 32;
    } else {
        r = 32;
    }
    if (r != last_r || g != last_g || b != last_b) {
        neopixelWrite(pins::kRgbLed, r, g, b);
        last_r = r;
        last_g = g;
        last_b = b;
    }
}

void stream_state(uint32_t now) {
    static uint32_t last = 0;
    if (!protocol.stream() || now - last < kStreamIntervalMs) return;
    last = now;
    char out[512];
    const size_t n = protocol.state_json(out, sizeof(out), now);
    Serial.write(reinterpret_cast<const uint8_t*>(out), n);
    Serial.write('\n');
}

void tick_motion(uint32_t dt_ms) {
    motion.tick(dt_ms);
    if (!enabled) return;
    for (uint8_t j = 0; j < profile.n_joints; ++j) {
        servo_output.write_us(profile.joints[j].channel, motion.joint(j).output_us());
    }
}

void poll_reboot(uint32_t now) {
    if (reboot_pending && static_cast<int32_t>(now - reboot_at_ms) >= 0) {
        ESP.restart();
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    pinMode(pins::kEstop, INPUT_PULLUP);
    delay(500);

    load_trims();

    // Seed the debouncer with the real boot state so it settles without a
    // phantom edge.
    estop_raw_prev = digitalRead(pins::kEstop) == HIGH;
    estop_edge_ms = millis();

    Serial.printf("{\"type\":\"hello\",\"fw\":\"%s\",\"proto\":%d,\"profile\":\"%s\",\"joints\":%d}\n",
                  ARM_FW_VERSION, ARM_PROTO_VERSION, profile.name, profile.n_joints);

    wifi_manager::begin();  // non-blocking: kicks off STA/AP, poll() drives it from loop()
    web_server::begin(protocol, profile);

    last_tick_ms = millis();
}

void loop() {
    poll_serial();

    const uint32_t now = millis();
    poll_estop(now);
    wifi_manager::poll(now);
    web_server::poll(now);

    const uint32_t dt = now - last_tick_ms;
    if (dt >= kTickIntervalMs) {
        last_tick_ms = now;
        tick_motion(dt);
    }

    stream_state(now);
    update_led(now);
    poll_reboot(now);
}
