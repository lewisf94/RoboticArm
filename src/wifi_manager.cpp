#include "wifi_manager.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>

#include <cstring>

namespace wifi_manager {

namespace {

constexpr uint32_t kStaTimeoutMs = 15000;
constexpr const char* kApSsid = "RoboArm-Setup";
constexpr const char* kApPass = "roboarm123";
constexpr uint8_t kApChannel = 1;
constexpr const char* kHostname = "roboarm";

struct Creds {
    char ssid[33] = "";
    char pass[65] = "";
};

Mode g_mode = Mode::kOff;
uint32_t g_sta_deadline_ms = 0;
bool g_mdns_started = false;

// isKey() (existence), not string length, decides "creds present": an empty
// ssid is rejected long before it would reach here (Protocol validates
// ssid_len >= 1), but an empty *pass* is a legitimate stored value (open
// network) and must not read as "no credentials".
bool load_creds(Creds& out) {
    Preferences prefs;
    if (!prefs.begin("net", /*readOnly=*/true)) return false;
    const bool has_ssid = prefs.isKey("ssid");
    if (has_ssid) {
        prefs.getString("ssid", out.ssid, sizeof(out.ssid));
        prefs.getString("pass", out.pass, sizeof(out.pass));
    }
    prefs.end();
    return has_ssid;
}

void start_mdns_once() {
    if (g_mdns_started) return;
    g_mdns_started = MDNS.begin(kHostname);
    Serial.println(g_mdns_started ? "# wifi: mDNS up, roboarm.local" : "# wifi: mDNS start failed");
}

void start_ap() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(kApSsid, kApPass, kApChannel);
    g_mode = Mode::kAp;
    Serial.printf("# wifi: AP mode, ssid=%s\n", kApSsid);
    start_mdns_once();
}

}  // namespace

void begin() {
    Creds creds;
    if (load_creds(creds)) {
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(kHostname);
        WiFi.begin(creds.ssid, creds.pass);
        g_mode = Mode::kConnectingSta;
        g_sta_deadline_ms = millis() + kStaTimeoutMs;
        Serial.printf("# wifi: connecting STA to %s\n", creds.ssid);
    } else {
        Serial.println("# wifi: no stored credentials");
        start_ap();
    }
}

void poll(uint32_t now_ms) {
    if (g_mode != Mode::kConnectingSta) return;

    if (WiFi.status() == WL_CONNECTED) {
        g_mode = Mode::kSta;
        Serial.printf("# wifi: STA connected, ip=%s\n", WiFi.localIP().toString().c_str());
        start_mdns_once();
        return;
    }
    // Signed-delta comparison so a millis() wraparound mid-attempt can't
    // make the deadline look like it's an epoch away instead of just passed.
    if (static_cast<int32_t>(now_ms - g_sta_deadline_ms) >= 0) {
        Serial.println("# wifi: STA connect timed out, falling back to AP");
        start_ap();
    }
}

Status status() {
    Status s;
    s.mode = g_mode;
    if (g_mode == Mode::kSta) {
        s.ip = WiFi.localIP();
        s.rssi = WiFi.RSSI();
    } else if (g_mode == Mode::kAp) {
        s.ip = WiFi.softAPIP();
    }
    return s;
}

bool save_creds(const char* ssid, const char* pass) {
    Preferences prefs;
    if (!prefs.begin("net", /*readOnly=*/false)) return false;
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();

    Creds check;
    return load_creds(check) && std::strcmp(check.ssid, ssid) == 0 && std::strcmp(check.pass, pass) == 0;
}

}  // namespace wifi_manager
