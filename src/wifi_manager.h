#pragma once

#include <cstdint>

#include <IPAddress.h>

// STA-with-AP-fallback WiFi state machine. Plain functions/struct (not a
// class) over module-local state - this firmware has exactly one radio, so
// there is nothing an instance would let a caller do that a singleton
// doesn't already. See docs/hardware.md for the AP name/password and
// docs/protocol.md for the `wifi` state field this backs.

namespace wifi_manager {

enum class Mode : uint8_t { kOff, kConnectingSta, kSta, kAp };

struct Status {
    Mode mode = Mode::kOff;
    IPAddress ip;  // 0.0.0.0 until an interface actually has one
    int rssi = 0;  // dBm; 0 outside kSta
};

// Call once from setup(), after Serial.begin(). Loads stored credentials; if
// present, starts a 15s STA connection attempt, otherwise goes straight to
// AP mode. Never blocks - the attempt is driven by poll().
void begin();

// Call every loop() iteration. Non-blocking: only compares millis() against
// a deadline and reads WiFi.status(), never delays.
void poll(uint32_t now_ms);

Status status();

// Persists credentials to NVS (namespace "net"). Does not itself apply them
// or reboot - the caller (Protocol's wifi_set handler, via a hook) is
// responsible for scheduling the reboot that makes them take effect.
// Verifies by reading back rather than trusting Preferences::putString's
// return value, which is ambiguous for an empty string (a legitimate value
// for `pass` - open networks - but indistinguishable from a failed write by
// byte count alone).
bool save_creds(const char* ssid, const char* pass);

}  // namespace wifi_manager
