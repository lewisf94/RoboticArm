#pragma once

#include <cstdint>

#include "arm_core/config.h"
#include "arm_core/file_store.h"
#include "arm_core/motion.h"

// Named poses: a snapshot of every joint's TARGET angle (not the live,
// mid-ease angle - a pose is a commanded configuration), saved/recalled by
// name. Persistence is abstracted behind IFileStore so this is host-
// testable (docs/architecture.md); the on-device backend (LittleFS) and the
// firmware wiring that actually calls persist()/load() arrive in T10.

namespace arm {

inline constexpr uint8_t kMaxPoses = 32;
inline constexpr uint8_t kMaxPoseNameLen = 16;

struct Pose {
    char name[kMaxPoseNameLen + 1] = "";
    float deg[kMaxJoints] = {};
};

enum class PoseResult : uint8_t { ok, bad_name, not_found, full };

class PoseStore {
public:
    // Captures motion.target(i) for i in [0, motion.n_joints()) under
    // `name`. Overwrites an existing pose of the same name in place (keeps
    // its slot/order); otherwise appends. Rejects a malformed name or a
    // full store without mutating anything.
    PoseResult save(const char* name, const MotionController& motion);

    // Null if no pose named `name` exists.
    const Pose* find(const char* name) const;

    // not_found leaves the store unchanged.
    PoseResult remove(const char* name);

    uint8_t count() const { return count_; }
    const Pose& at(uint8_t i) const { return poses_[i]; }

    // JSON file, shape {"poses":[{"name":…,"deg":[…]}]}.
    // persist() writes exactly n_joints angles per pose - poses.json is
    // never meant to be portable across profiles - and never writes a
    // truncated file (fails closed if it wouldn't fit).
    // load() fully replaces the in-memory set: on any failure (missing
    // file, corrupt JSON, no "poses" array) it ends empty-but-usable, not
    // partially populated. Per-entry: a pose whose `deg` length doesn't
    // match n_joints, or whose name fails the same check save() uses, is
    // dropped rather than aborting the whole load (CLAUDE.md: never trust
    // a stored file). Angles are NOT clamped here - clamping happens where
    // a pose is actually applied (Protocol::cmd_goto_pose).
    bool persist(IFileStore& store, uint8_t n_joints) const;
    bool load(IFileStore& store, uint8_t n_joints);

private:
    static bool valid_name(const char* name);
    int find_index(const char* name) const;

    Pose poses_[kMaxPoses];
    uint8_t count_ = 0;
};

}  // namespace arm
