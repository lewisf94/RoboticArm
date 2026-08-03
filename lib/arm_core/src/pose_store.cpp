#include "arm_core/pose_store.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <ArduinoJson.h>

using namespace ArduinoJson;

namespace arm {

namespace {

// Same pattern as protocol.cpp's BoundedAllocator (see that file for the
// full rationale) - a plain malloc/realloc/free allocator with a hard byte
// cap, kept as its own local copy here rather than shared, since exporting
// it would mean putting an ArduinoJson::Allocator subclass in a public
// header for no real benefit. Sized to comfortably hold kMaxPoses entries'
// worth of ArduinoJson DOM overhead (a JSON value costs more as a DOM node
// than as printed text) and to clear v7's first fixed pool-chunk
// allocation.
class BoundedAllocator : public Allocator {
public:
    explicit BoundedAllocator(size_t cap) : cap_(cap) {}

    void* allocate(size_t size) override {
        if (used_ + size > cap_) return nullptr;
        void* raw = std::malloc(size + kHeaderBytes);
        if (!raw) return nullptr;
        *static_cast<size_t*>(raw) = size;
        used_ += size;
        return static_cast<uint8_t*>(raw) + kHeaderBytes;
    }

    void deallocate(void* ptr) override {
        if (!ptr) return;
        void* raw = static_cast<uint8_t*>(ptr) - kHeaderBytes;
        used_ -= *static_cast<size_t*>(raw);
        std::free(raw);
    }

    void* reallocate(void* ptr, size_t new_size) override {
        if (!ptr) return allocate(new_size);
        void* old_raw = static_cast<uint8_t*>(ptr) - kHeaderBytes;
        const size_t old_size = *static_cast<size_t*>(old_raw);
        if (used_ - old_size + new_size > cap_) return nullptr;
        void* new_raw = std::realloc(old_raw, new_size + kHeaderBytes);
        if (!new_raw) return nullptr;
        *static_cast<size_t*>(new_raw) = new_size;
        used_ = used_ - old_size + new_size;
        return static_cast<uint8_t*>(new_raw) + kHeaderBytes;
    }

private:
    static constexpr size_t kHeaderBytes = 16;
    size_t cap_;
    size_t used_ = 0;
};

constexpr size_t kJsonPoolBytes = 12288;  // DOM budget for up to kMaxPoses entries
constexpr size_t kJsonTextBytes = 4096;   // serialized-text budget, same worst case
constexpr const char* kPosesPath = "/data/poses.json";

}  // namespace

bool PoseStore::valid_name(const char* name) {
    if (!name) return false;
    const size_t len = std::strlen(name);
    if (len < 1 || len > kMaxPoseNameLen) return false;
    for (size_t i = 0; i < len; ++i) {
        const char c = name[i];
        const bool ok =
            (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

int PoseStore::find_index(const char* name) const {
    for (uint8_t i = 0; i < count_; ++i) {
        if (std::strcmp(poses_[i].name, name) == 0) return i;
    }
    return -1;
}

PoseResult PoseStore::save(const char* name, const MotionController& motion) {
    if (!valid_name(name)) return PoseResult::bad_name;

    int idx = find_index(name);
    if (idx < 0) {
        if (count_ >= kMaxPoses) return PoseResult::full;
        idx = count_++;
        std::snprintf(poses_[idx].name, sizeof(poses_[idx].name), "%s", name);
    }
    const uint8_t n = motion.n_joints();
    for (uint8_t i = 0; i < n; ++i) poses_[idx].deg[i] = motion.target(i);
    return PoseResult::ok;
}

const Pose* PoseStore::find(const char* name) const {
    const int idx = find_index(name);
    return idx < 0 ? nullptr : &poses_[idx];
}

PoseResult PoseStore::remove(const char* name) {
    const int idx = find_index(name);
    if (idx < 0) return PoseResult::not_found;
    for (uint8_t i = static_cast<uint8_t>(idx); i + 1 < count_; ++i) poses_[i] = poses_[i + 1];
    --count_;
    return PoseResult::ok;
}

bool PoseStore::persist(IFileStore& store, uint8_t n_joints) const {
    BoundedAllocator alloc(kJsonPoolBytes);
    JsonDocument doc(&alloc);
    JsonArray poses = doc["poses"].to<JsonArray>();
    for (uint8_t i = 0; i < count_; ++i) {
        JsonObject o = poses.add<JsonObject>();
        o["name"] = poses_[i].name;
        JsonArray deg = o["deg"].to<JsonArray>();
        for (uint8_t j = 0; j < n_joints; ++j) deg.add(poses_[i].deg[j]);
    }

    // measureJson (not a serializeJson truncation check) so a too-big
    // document fails closed instead of ever writing a truncated,
    // unparseable file - unlike protocol.cpp's reply buffers, this one
    // gets read back later.
    if (measureJson(doc) >= kJsonTextBytes) return false;

    char buf[kJsonTextBytes];
    const size_t len = serializeJson(doc, buf, sizeof(buf));
    return store.write(kPosesPath, buf, len);
}

bool PoseStore::load(IFileStore& store, uint8_t n_joints) {
    count_ = 0;  // load() always fully replaces the set; any failure below -> empty-but-usable

    char buf[kJsonTextBytes];
    size_t len = 0;
    if (!store.read(kPosesPath, buf, sizeof(buf), len)) return false;

    BoundedAllocator alloc(kJsonPoolBytes);
    JsonDocument doc(&alloc);
    if (deserializeJson(doc, buf, len) != DeserializationError::Ok) return false;

    JsonArrayConst poses = doc["poses"].as<JsonArrayConst>();
    if (poses.isNull()) return false;

    uint8_t new_count = 0;
    for (JsonObjectConst o : poses) {
        if (new_count >= kMaxPoses) break;

        const char* name = o["name"] | "";
        if (!valid_name(name)) continue;

        JsonArrayConst deg = o["deg"].as<JsonArrayConst>();
        if (deg.isNull() || deg.size() != n_joints) continue;

        Pose& p = poses_[new_count];
        std::snprintf(p.name, sizeof(p.name), "%s", name);
        uint8_t j = 0;
        for (JsonVariantConst v : deg) {
            p.deg[j] = v.as<float>();
            ++j;
        }
        ++new_count;
    }
    count_ = new_count;
    return true;
}

}  // namespace arm
