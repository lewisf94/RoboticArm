"""Pure-Python codec for the RoboticArm serial protocol (docs/protocol.md).

Deliberately free of rclpy and serial imports (CLAUDE.md rule) so `colcon
test` can exercise every line of it with no ROS middleware and no device
attached. bridge.py owns all I/O and ROS types; this module only turns lines
of text into data and back.

Unit boundary: the device speaks degrees and millimetres, ROS speaks radians
and metres. **Every** conversion the bridge performs lives in this file.

Parsing is deliberately forgiving in one direction only: any line that isn't
a well-formed protocol message parses to None so the caller can skip it (the
firmware also emits `#`-prefixed human log lines, and a half-written line is
normal right after a hot-plug). It is never forgiving about *values* - a
state line whose `j` array isn't numeric is discarded rather than
half-published.
"""

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
MM2M = 0.001


def deg2rad(deg: float) -> float:
    return deg * DEG2RAD


def rad2deg(rad: float) -> float:
    return rad * RAD2DEG


def mm2m(mm: float) -> float:
    return mm * MM2M


# --------------------------------------------------------------------------
# Parsed message types
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class JointInfo:
    name: str
    min_deg: float
    max_deg: float
    home_deg: float
    vmax_dps: float
    is_gripper: bool = False

    @property
    def min_rad(self) -> float:
        return deg2rad(self.min_deg)

    @property
    def max_rad(self) -> float:
        return deg2rad(self.max_deg)


@dataclass(frozen=True)
class Profile:
    """Joint set reported by `get_profile` - the bridge's map between ROS
    joint names and device array indices."""

    name: str
    joints: Tuple[JointInfo, ...]
    fw: str = ""
    proto: int = 0

    @property
    def names(self) -> List[str]:
        return [j.name for j in self.joints]

    @property
    def n_joints(self) -> int:
        return len(self.joints)

    def index_of(self, name: str) -> Optional[int]:
        for i, j in enumerate(self.joints):
            if j.name == name:
                return i
        return None


@dataclass(frozen=True)
class Hello:
    fw: str
    proto: int
    profile: str
    joints: int


@dataclass(frozen=True)
class State:
    t_ms: int
    enabled: bool
    positions_deg: Tuple[float, ...]
    targets_deg: Tuple[float, ...] = ()
    pose_mm: Optional[Dict[str, float]] = None
    seq: Optional[Dict[str, Any]] = None
    heap: int = 0

    @property
    def positions_rad(self) -> List[float]:
        return [deg2rad(d) for d in self.positions_deg]

    @property
    def targets_rad(self) -> List[float]:
        return [deg2rad(d) for d in self.targets_deg]

    @property
    def pose_m(self) -> Optional[Dict[str, float]]:
        """Tool pose with x/y/z converted mm -> m; `pitch` stays an angle and
        is converted deg -> rad. None until the device implements FK (T13)."""
        if self.pose_mm is None:
            return None
        out: Dict[str, float] = {}
        for key in ("x", "y", "z"):
            if key in self.pose_mm:
                out[key] = mm2m(self.pose_mm[key])
        if "pitch" in self.pose_mm:
            out["pitch"] = deg2rad(self.pose_mm["pitch"])
        return out


@dataclass(frozen=True)
class Ack:
    cmd: str
    id: Optional[int]
    data: Dict[str, Any]


@dataclass(frozen=True)
class Err:
    cmd: str
    id: Optional[int]
    code: str
    msg: str


# --------------------------------------------------------------------------
# Parsing
# --------------------------------------------------------------------------


def _is_number(v: Any) -> bool:
    # bool is a subclass of int in Python; a JSON true must not pass as 1.0.
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _float_list(v: Any) -> Optional[List[float]]:
    if not isinstance(v, list):
        return None
    out: List[float] = []
    for x in v:
        if not _is_number(x):
            return None
        out.append(float(x))
    return out


def _opt_id(obj: Dict[str, Any]) -> Optional[int]:
    v = obj.get("id")
    return int(v) if _is_number(v) else None


def parse_line(line: str) -> Optional[Any]:
    """Parse one protocol line into Hello/State/Ack/Err, or None if the line
    is not a protocol message (blank, a `#` log line, malformed JSON, an
    unknown `type`, or structurally invalid). Never raises."""
    if not isinstance(line, str):
        return None
    text = line.strip()
    if not text or text.startswith("#"):
        return None
    try:
        obj = json.loads(text)
    except (ValueError, TypeError):
        return None
    if not isinstance(obj, dict):
        return None

    kind = obj.get("type")

    if kind == "state":
        positions = _float_list(obj.get("j"))
        if positions is None:
            return None  # `j` is the whole point of a state line
        targets = _float_list(obj.get("tgt"))
        pose = obj.get("pose")
        seq = obj.get("seq")
        return State(
            t_ms=int(obj["t"]) if _is_number(obj.get("t")) else 0,
            enabled=bool(obj.get("en", False)),
            positions_deg=tuple(positions),
            targets_deg=tuple(targets) if targets is not None else (),
            pose_mm=pose if isinstance(pose, dict) else None,
            seq=seq if isinstance(seq, dict) else None,
            heap=int(obj["heap"]) if _is_number(obj.get("heap")) else 0,
        )

    if kind == "hello":
        return Hello(
            fw=str(obj.get("fw", "")),
            proto=int(obj["proto"]) if _is_number(obj.get("proto")) else 0,
            profile=str(obj.get("profile", "")),
            joints=int(obj["joints"]) if _is_number(obj.get("joints")) else 0,
        )

    if kind == "ack":
        return Ack(cmd=str(obj.get("cmd", "")), id=_opt_id(obj), data=obj)

    if kind == "err":
        return Err(
            cmd=str(obj.get("cmd", "")),
            id=_opt_id(obj),
            code=str(obj.get("code", "")),
            msg=str(obj.get("msg", "")),
        )

    return None


def parse_profile(ack: Dict[str, Any]) -> Optional[Profile]:
    """Build a Profile from a `get_profile` ack payload (docs/protocol.md).
    Returns None if the payload has no usable joint list."""
    if not isinstance(ack, dict):
        return None
    raw = ack.get("joints")
    if not isinstance(raw, list) or not raw:
        return None

    joints: List[JointInfo] = []
    for entry in raw:
        if not isinstance(entry, dict) or not isinstance(entry.get("name"), str):
            return None
        joints.append(
            JointInfo(
                name=entry["name"],
                min_deg=float(entry.get("min", 0.0)) if _is_number(entry.get("min")) else 0.0,
                max_deg=float(entry.get("max", 0.0)) if _is_number(entry.get("max")) else 0.0,
                home_deg=float(entry.get("home", 0.0)) if _is_number(entry.get("home")) else 0.0,
                vmax_dps=float(entry.get("vmax", 0.0)) if _is_number(entry.get("vmax")) else 0.0,
                is_gripper=bool(entry.get("gripper", False)),
            )
        )

    return Profile(
        name=str(ack.get("name", "")),
        joints=tuple(joints),
        fw=str(ack.get("fw", "")),
        proto=int(ack["proto"]) if _is_number(ack.get("proto")) else 0,
    )


# --------------------------------------------------------------------------
# Command building
# --------------------------------------------------------------------------


def _line(obj: Dict[str, Any]) -> str:
    """Serialize one command. No trailing newline - framing belongs to the
    transport."""
    return json.dumps(obj, separators=(",", ":"))


def _with_id(obj: Dict[str, Any], req_id: Optional[int]) -> Dict[str, Any]:
    if req_id is not None:
        obj["id"] = req_id
    return obj


def build_get_profile(req_id: Optional[int] = None) -> str:
    return _line(_with_id({"cmd": "get_profile"}, req_id))


def build_get_state(req_id: Optional[int] = None) -> str:
    return _line(_with_id({"cmd": "get_state"}, req_id))


def build_stream(on: bool, req_id: Optional[int] = None) -> str:
    return _line(_with_id({"cmd": "stream", "on": bool(on)}, req_id))


def build_enable(on: bool, req_id: Optional[int] = None) -> str:
    return _line(_with_id({"cmd": "enable", "on": bool(on)}, req_id))


def build_estop(req_id: Optional[int] = None) -> str:
    return _line(_with_id({"cmd": "estop"}, req_id))


def build_home(dur_ms: Optional[int] = None, req_id: Optional[int] = None) -> str:
    obj: Dict[str, Any] = {"cmd": "home"}
    if dur_ms is not None and dur_ms > 0:
        obj["dur"] = int(dur_ms)
    return _line(_with_id(obj, req_id))


def build_set_joints(targets_deg: Sequence[Optional[float]], req_id: Optional[int] = None) -> str:
    """`deg` must be exactly n_joints long; None entries serialize to JSON
    null, which the device reads as "leave this joint alone"."""
    deg = [None if d is None else float(d) for d in targets_deg]
    return _line(_with_id({"cmd": "set_joints", "deg": deg}, req_id))


def map_targets(
    profile: Profile, names: Sequence[str], positions_rad: Sequence[float]
) -> Tuple[List[Optional[float]], List[str]]:
    """Turn a (possibly partial, possibly reordered) ROS JointState into the
    device's positional `deg` array.

    Returns (targets_deg, unknown_names): targets_deg is always n_joints long
    with None for every joint the message didn't mention; unknown_names holds
    names that aren't in the profile at all so the caller can warn about a
    typo instead of silently dropping a command.

    Trailing names without a matching position are ignored - a JointState
    carrying names but no positions (a velocity-only message) is not a target
    command.
    """
    targets: List[Optional[float]] = [None] * profile.n_joints
    unknown: List[str] = []

    for name, pos in zip(names, positions_rad):
        idx = profile.index_of(name)
        if idx is None:
            unknown.append(name)
            continue
        targets[idx] = rad2deg(float(pos))

    return targets, unknown
