#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

// Persistence backend abstraction so anything built on top (PoseStore, and
// later the sequencer) is host-testable without a real filesystem. The
// on-device implementation (LittleFS) arrives in T10; MemFileStore below is
// the host-test backend, trivial enough to live here rather than duplicated
// per test file.

namespace arm {

class IFileStore {
public:
    // Reads the whole file at `path` into `buf` (capacity `cap`), setting
    // `len` to the number of bytes read. False if the file doesn't exist,
    // or is larger than `cap` (never partially fills `buf` in that case).
    virtual bool read(const char* path, char* buf, size_t cap, size_t& len) = 0;

    // Writes `buf` (length `len`) to `path`, replacing any existing content.
    virtual bool write(const char* path, const char* buf, size_t len) = 0;

    virtual ~IFileStore() = default;
};

// Fixed table of named byte buffers standing in for a filesystem - no
// directories, no partial writes, nothing persists past the process. Used
// by native tests only.
class MemFileStore : public IFileStore {
public:
    bool read(const char* path, char* buf, size_t cap, size_t& len) override {
        Entry* e = find(path);
        if (!e || e->len > cap) return false;
        std::memcpy(buf, e->data, e->len);
        len = e->len;
        return true;
    }

    bool write(const char* path, const char* buf, size_t len) override {
        if (len > kMaxFileBytes) return false;
        Entry* e = find(path);
        if (!e) {
            e = free_slot();
            if (!e) return false;  // table full
            std::snprintf(e->path, sizeof(e->path), "%s", path);
            e->in_use = true;
        }
        std::memcpy(e->data, buf, len);
        e->len = len;
        return true;
    }

private:
    static constexpr uint8_t kMaxFiles = 4;
    static constexpr size_t kMaxFileBytes = 6144;

    struct Entry {
        char path[32] = "";
        char data[kMaxFileBytes] = {};
        size_t len = 0;
        bool in_use = false;
    };

    Entry* find(const char* path) {
        for (Entry& e : files_) {
            if (e.in_use && std::strcmp(e.path, path) == 0) return &e;
        }
        return nullptr;
    }

    Entry* free_slot() {
        for (Entry& e : files_) {
            if (!e.in_use) return &e;
        }
        return nullptr;
    }

    Entry files_[kMaxFiles];
};

}  // namespace arm
