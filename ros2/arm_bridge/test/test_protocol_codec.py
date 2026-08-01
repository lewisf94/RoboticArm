"""Unit tests for the pure-Python protocol codec.

No rclpy, no serial, no device: these run anywhere pytest does, which is the
whole reason the codec is a separate module (CLAUDE.md).
"""

import json
import math

import pytest

from arm_bridge.protocol_codec import (
    Ack,
    Err,
    Hello,
    JointInfo,
    Profile,
    State,
    build_enable,
    build_estop,
    build_get_profile,
    build_home,
    build_set_joints,
    build_stream,
    deg2rad,
    map_targets,
    parse_line,
    parse_profile,
    rad2deg,
)

BENCH = Profile(
    name="bench_3dof",
    joints=(
        JointInfo("base", -90.0, 90.0, 0.0, 120.0),
        JointInfo("shoulder", 0.0, 120.0, 60.0, 120.0),
        JointInfo("grip", 0.0, 60.0, 30.0, 120.0, is_gripper=True),
    ),
)


# --------------------------------------------------------------------------
# Parsing: state
# --------------------------------------------------------------------------


def test_parse_state_basic():
    line = json.dumps(
        {
            "type": "state",
            "t": 123456,
            "en": True,
            "j": [45.0, 10.2, -30.0],
            "tgt": [45.0, 20.0, -30.0],
            "pose": None,
            "seq": None,
            "heap": 200000,
        }
    )
    msg = parse_line(line)
    assert isinstance(msg, State)
    assert msg.t_ms == 123456
    assert msg.enabled is True
    assert msg.positions_deg == (45.0, 10.2, -30.0)
    assert msg.targets_deg == (45.0, 20.0, -30.0)
    assert msg.pose_mm is None
    assert msg.seq is None
    assert msg.heap == 200000


def test_parse_state_converts_to_radians():
    msg = parse_line('{"type":"state","en":false,"j":[45,90,-180],"tgt":[0,0,0]}')
    assert msg.positions_rad == pytest.approx([math.pi / 4, math.pi / 2, -math.pi])


def test_parse_state_tolerates_missing_optional_fields():
    # `j` is the only field /joint_states actually needs; a firmware that
    # omits tgt/heap/t must still drive the model rather than go silent.
    msg = parse_line('{"type":"state","j":[1,2,3]}')
    assert isinstance(msg, State)
    assert msg.positions_deg == (1.0, 2.0, 3.0)
    assert msg.targets_deg == ()
    assert msg.enabled is False
    assert msg.t_ms == 0


def test_parse_state_pose_mm_to_metres():
    msg = parse_line(
        '{"type":"state","j":[0,0,0],"tgt":[0,0,0],'
        '"pose":{"x":260.0,"y":0.0,"z":60.0,"pitch":90.0}}'
    )
    pose = msg.pose_m
    assert pose["x"] == pytest.approx(0.260)
    assert pose["z"] == pytest.approx(0.060)
    assert pose["pitch"] == pytest.approx(math.pi / 2)


# --------------------------------------------------------------------------
# Parsing: malformed input must be skipped, never raise
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "line",
    [
        "",
        "   ",
        "not json at all",
        "{",
        '{"type":"state","j":[1,2,',       # truncated (hot-plug mid-line)
        "[1,2,3]",                          # valid JSON, not an object
        '"a string"',
        "null",
        '{"type":"state"}',                # no j array
        '{"type":"state","j":"nope"}',     # j not a list
        '{"type":"state","j":[1,"x",3]}',  # j not all numeric
        '{"type":"state","j":[1,true,3]}', # JSON true must not pass as 1.0
        '{"type":"unknown_kind","j":[1]}',
        '{"no_type":1}',
        "# firmware log line",
        "#",
    ],
)
def test_parse_line_rejects_garbage(line):
    assert parse_line(line) is None


def test_parse_line_rejects_non_string():
    assert parse_line(None) is None
    assert parse_line(42) is None


# --------------------------------------------------------------------------
# Parsing: hello / ack / err
# --------------------------------------------------------------------------


def test_parse_hello():
    msg = parse_line('{"type":"hello","fw":"0.1.0","proto":1,"profile":"bench_3dof","joints":3}')
    assert isinstance(msg, Hello)
    assert (msg.fw, msg.proto, msg.profile, msg.joints) == ("0.1.0", 1, "bench_3dof", 3)


def test_parse_ack_with_and_without_id():
    msg = parse_line('{"type":"ack","cmd":"enable","id":7}')
    assert isinstance(msg, Ack)
    assert msg.cmd == "enable"
    assert msg.id == 7

    msg = parse_line('{"type":"ack","cmd":"estop"}')
    assert isinstance(msg, Ack)
    assert msg.id is None


def test_parse_err():
    msg = parse_line(
        '{"type":"err","cmd":"enable","id":3,"code":"disabled","msg":"estop pin active"}'
    )
    assert isinstance(msg, Err)
    assert (msg.cmd, msg.id, msg.code) == ("enable", 3, "disabled")
    assert msg.msg == "estop pin active"


# --------------------------------------------------------------------------
# Profile parsing
# --------------------------------------------------------------------------


def test_parse_profile_from_get_profile_ack():
    ack = json.loads(
        '{"type":"ack","cmd":"get_profile","name":"bench_3dof",'
        '"joints":[{"name":"base","min":-90,"max":90,"home":0,"vmax":120,"gripper":false},'
        '{"name":"shoulder","min":0,"max":120,"home":60,"vmax":120,"gripper":false},'
        '{"name":"grip","min":0,"max":60,"home":30,"vmax":120,"gripper":true}],'
        '"geo":{"base_h":60,"r_off":0,"L1":120,"L2":140,"Lw":0,"wrist":false},'
        '"fw":"0.1.0","proto":1}'
    )
    profile = parse_profile(ack)
    assert profile is not None
    assert profile.name == "bench_3dof"
    assert profile.n_joints == 3
    assert profile.names == ["base", "shoulder", "grip"]
    assert profile.joints[2].is_gripper is True
    assert profile.joints[0].min_rad == pytest.approx(-math.pi / 2)
    assert profile.index_of("shoulder") == 1
    assert profile.index_of("nonexistent") is None


@pytest.mark.parametrize("payload", [{}, {"joints": []}, {"joints": "nope"}, {"joints": [{}]}])
def test_parse_profile_rejects_unusable_payloads(payload):
    assert parse_profile(payload) is None


# --------------------------------------------------------------------------
# Target mapping: the "omitted joints" rule
# --------------------------------------------------------------------------


def test_map_targets_all_joints():
    targets, unknown = map_targets(
        BENCH, ["base", "shoulder", "grip"], [math.pi / 4, math.pi / 3, 0.0]
    )
    assert unknown == []
    assert targets == pytest.approx([45.0, 60.0, 0.0])


def test_map_targets_omitted_joints_become_none():
    targets, unknown = map_targets(BENCH, ["shoulder"], [math.pi / 6])
    assert unknown == []
    assert targets[0] is None
    assert targets[1] == pytest.approx(30.0)
    assert targets[2] is None
    assert len(targets) == BENCH.n_joints


def test_map_targets_respects_message_order_not_array_order():
    # A JointState may list joints in any order; mapping is by name.
    targets, _ = map_targets(BENCH, ["grip", "base"], [deg2rad(15.0), deg2rad(-30.0)])
    assert targets[0] == pytest.approx(-30.0)
    assert targets[1] is None
    assert targets[2] == pytest.approx(15.0)


def test_map_targets_reports_unknown_names():
    targets, unknown = map_targets(BENCH, ["elbow", "base"], [1.0, 0.0])
    assert unknown == ["elbow"]
    assert targets[0] == pytest.approx(0.0)


def test_map_targets_ignores_names_without_positions():
    # velocity-only JointState: names present, positions empty -> no command
    targets, unknown = map_targets(BENCH, ["base", "shoulder"], [])
    assert targets == [None, None, None]
    assert unknown == []


def test_map_targets_empty_message():
    targets, unknown = map_targets(BENCH, [], [])
    assert targets == [None, None, None]
    assert unknown == []


# --------------------------------------------------------------------------
# Command building
# --------------------------------------------------------------------------


def test_build_set_joints_emits_null_for_omitted():
    line = build_set_joints([45.0, None, 0.0])
    obj = json.loads(line)
    assert obj["cmd"] == "set_joints"
    assert obj["deg"] == [45.0, None, 0.0]
    assert "null" in line  # the device's "leave this joint alone" sentinel


def test_build_commands_shapes():
    assert json.loads(build_get_profile()) == {"cmd": "get_profile"}
    assert json.loads(build_stream(True)) == {"cmd": "stream", "on": True}
    assert json.loads(build_enable(False)) == {"cmd": "enable", "on": False}
    assert json.loads(build_estop()) == {"cmd": "estop"}
    assert json.loads(build_home()) == {"cmd": "home"}
    assert json.loads(build_home(dur_ms=1500)) == {"cmd": "home", "dur": 1500}
    # dur is omitted rather than sent as 0, which the device would treat as
    # "compute duration from vmax" anyway - but explicit beats implicit.
    assert json.loads(build_home(dur_ms=0)) == {"cmd": "home"}


def test_build_commands_echo_id():
    assert json.loads(build_enable(True, req_id=9))["id"] == 9
    assert json.loads(build_estop(req_id=2))["id"] == 2
    assert "id" not in json.loads(build_estop())


def test_built_lines_are_single_line_json():
    # The transport frames on '\n'; an embedded newline would split one
    # command into two malformed ones.
    for line in [
        build_set_joints([1.0, None, 2.0], req_id=1),
        build_home(1000),
        build_enable(True),
    ]:
        assert "\n" not in line
        json.loads(line)  # round-trips


# --------------------------------------------------------------------------
# Unit conversion
# --------------------------------------------------------------------------


@pytest.mark.parametrize("deg", [0.0, 45.0, -90.0, 120.0, 359.9])
def test_deg_rad_round_trip(deg):
    assert rad2deg(deg2rad(deg)) == pytest.approx(deg)


def test_known_conversions():
    assert deg2rad(180.0) == pytest.approx(math.pi)
    assert rad2deg(math.pi / 2) == pytest.approx(90.0)


def test_state_to_target_round_trip_through_codec():
    """A state line's radians, fed straight back as a target, must produce the
    same degrees the device reported - the bridge must not drift."""
    state = parse_line('{"type":"state","en":true,"j":[45.0,60.0,30.0],"tgt":[0,0,0]}')
    targets, unknown = map_targets(BENCH, BENCH.names, state.positions_rad)
    assert unknown == []
    assert targets == pytest.approx([45.0, 60.0, 30.0])
    assert json.loads(build_set_joints(targets))["deg"] == pytest.approx([45.0, 60.0, 30.0])
