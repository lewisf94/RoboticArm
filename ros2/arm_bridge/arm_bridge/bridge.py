"""ROS 2 <-> RoboticArm serial bridge.

A translator and nothing more (CLAUDE.md): joint limits, easing, e-stop
latching and every other safety behaviour live in arm_core on the device.
This node converts message formats and units, and is free to be restarted,
killed or disconnected at any moment without changing what the arm will
accept.

    /joint_states            <- device `state` lines (radians)
    /arm/enabled             <- device `state` lines
    /arm/target_joint_states -> `set_joints`
    /arm/enable  (SetBool)   -> `enable`
    /arm/estop   (Trigger)   -> `estop`
    /arm/home    (Trigger)   -> `home`

Threading: deliberately single-threaded. Serial is drained from one timer,
and the service handlers reuse that same drain loop while waiting for their
ack (`_send_and_wait`), so a handler can block briefly without deadlocking
against a reader running in another thread - there isn't one.
"""

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from arm_bridge import protocol_codec as codec
from arm_bridge.serial_transport import SerialLineTransport

# Targets are coalesced rather than dropped: a publisher faster than this
# would otherwise flood the 115200 link, but the *latest* target must always
# reach the device or the arm stops short of where it was told to go.
TARGET_MIN_INTERVAL_S = 0.05  # 20 Hz
POLL_INTERVAL_S = 0.01
RECONNECT_BACKOFF_S = (0.5, 1.0, 2.0, 4.0)
ACK_TIMEOUT_S = 1.0


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self._port = self.get_parameter("port").get_parameter_value().string_value
        self._baud = self.get_parameter("baud").get_parameter_value().integer_value

        self._transport: Optional[SerialLineTransport] = None
        self._profile: Optional[codec.Profile] = None
        self._next_id = 1
        self._backoff_idx = 0
        self._next_connect_at = 0.0

        self._pending_target: Optional[List[Optional[float]]] = None
        self._last_target_sent_at = 0.0
        self._last_enabled: Optional[bool] = None

        self._pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self._pub_enabled = self.create_publisher(Bool, "/arm/enabled", 10)
        self.create_subscription(
            JointState, "/arm/target_joint_states", self._on_target, 10
        )
        self.create_service(SetBool, "/arm/enable", self._on_enable)
        self.create_service(Trigger, "/arm/estop", self._on_estop)
        self.create_service(Trigger, "/arm/home", self._on_home)

        self.create_timer(POLL_INTERVAL_S, self._poll)
        self.get_logger().info(f"arm_bridge starting on {self._port} @ {self._baud}")

    # -- connection ---------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._transport is not None

    def _connect(self) -> None:
        try:
            self._transport = SerialLineTransport(self._port, self._baud)
        except Exception as exc:  # noqa: BLE001 - any port failure retries
            self._schedule_reconnect()
            self.get_logger().warning(f"open {self._port} failed: {exc}")
            return

        self.get_logger().info(f"opened {self._port}")
        self._backoff_idx = 0

        # Handshake: learn the joint names, then ask for telemetry. The device
        # sends `hello` unprompted on boot, but we may well have connected to
        # an already-running board, so the profile is fetched explicitly
        # rather than waited for.
        profile_ack = self._send_and_wait(codec.build_get_profile, "get_profile")
        if isinstance(profile_ack, codec.Ack):
            self._profile = codec.parse_profile(profile_ack.data)

        if self._profile is None:
            self.get_logger().error("no usable profile from get_profile; retrying")
            self._disconnect()
            return

        self.get_logger().info(
            f"profile '{self._profile.name}': {', '.join(self._profile.names)}"
        )
        self._send_and_wait(lambda i: codec.build_stream(True, i), "stream")

    def _disconnect(self) -> None:
        if self._transport is not None:
            self._transport.close()
        self._transport = None
        self._profile = None
        self._pending_target = None
        # Deliberately do NOT republish the last known joint states: a stale
        # model in RViz that looks live is worse than one that stops updating.
        self._last_enabled = None
        self._schedule_reconnect()

    def _schedule_reconnect(self) -> None:
        delay = RECONNECT_BACKOFF_S[min(self._backoff_idx, len(RECONNECT_BACKOFF_S) - 1)]
        self._backoff_idx += 1
        self._next_connect_at = time.monotonic() + delay

    # -- serial I/O ---------------------------------------------------------

    def _take_id(self) -> int:
        req_id = self._next_id
        self._next_id += 1
        return req_id

    def _write(self, line: str) -> bool:
        if self._transport is None:
            return False
        try:
            self._transport.write_line(line)
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"write failed: {exc}")
            self._disconnect()
            return False

    def _read_lines(self) -> List[str]:
        if self._transport is None:
            return []
        try:
            return self._transport.read_available_lines()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"read failed: {exc}")
            self._disconnect()
            return []

    def _send_and_wait(self, build, cmd_name: str, timeout: float = ACK_TIMEOUT_S):
        """Send a command and drain serial until its ack/err arrives.

        Safe to call from a service handler: this is the same thread the poll
        timer runs on, so nothing else is reading the port concurrently, and
        every unrelated line seen while waiting is processed normally rather
        than discarded.
        """
        req_id = self._take_id()
        if not self._write(build(req_id)):
            return None

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for line in self._read_lines():
                msg = codec.parse_line(line)
                if isinstance(msg, (codec.Ack, codec.Err)) and msg.id == req_id:
                    return msg
                self._handle(msg)
            if not self.connected:
                return None
            time.sleep(0.002)

        self.get_logger().warning(f"{cmd_name}: no reply within {timeout:.1f}s")
        return None

    # -- message handling ---------------------------------------------------

    def _handle(self, msg) -> None:
        if isinstance(msg, codec.State):
            self._publish_state(msg)
        elif isinstance(msg, codec.Hello):
            self.get_logger().info(f"device hello: fw {msg.fw}, profile {msg.profile}")
        elif isinstance(msg, codec.Err):
            self.get_logger().warning(f"device err [{msg.cmd}] {msg.code}: {msg.msg}")

    def _publish_state(self, state: codec.State) -> None:
        if self._profile is None:
            return
        if len(state.positions_deg) != self._profile.n_joints:
            self.get_logger().warning(
                f"state has {len(state.positions_deg)} joints, "
                f"profile has {self._profile.n_joints}; skipping"
            )
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self._profile.names
        js.position = state.positions_rad
        self._pub_joint_states.publish(js)

        if state.enabled != self._last_enabled:
            self._last_enabled = state.enabled
            self._pub_enabled.publish(Bool(data=state.enabled))

    # -- ROS callbacks ------------------------------------------------------

    def _on_target(self, msg: JointState) -> None:
        if self._profile is None:
            self.get_logger().warning("target ignored: not connected")
            return

        targets, unknown = codec.map_targets(self._profile, msg.name, msg.position)
        if unknown:
            self.get_logger().warning(f"unknown joint name(s) ignored: {unknown}")
        if all(t is None for t in targets):
            return  # nothing addressed to this arm

        self._pending_target = targets  # coalesced; sent by _poll

    def _on_enable(self, request, response):
        reply = self._send_and_wait(
            lambda i: codec.build_enable(request.data, i), "enable"
        )
        response.success = isinstance(reply, codec.Ack)
        if isinstance(reply, codec.Err):
            response.message = f"{reply.code}: {reply.msg}"
        elif reply is None:
            response.message = "no reply from device"
        else:
            response.message = "enabled" if request.data else "disabled"
        return response

    def _on_estop(self, request, response):
        del request
        # Bypasses the target coalescer entirely, and drops any target still
        # waiting so the next poll can't push motion at a just-stopped arm.
        self._pending_target = None
        reply = self._send_and_wait(codec.build_estop, "estop")
        response.success = isinstance(reply, codec.Ack)
        response.message = "estopped" if response.success else "no reply from device"
        return response

    def _on_home(self, request, response):
        del request
        reply = self._send_and_wait(lambda i: codec.build_home(req_id=i), "home")
        response.success = isinstance(reply, codec.Ack)
        if isinstance(reply, codec.Err):
            response.message = f"{reply.code}: {reply.msg}"
        else:
            response.message = "homing" if response.success else "no reply from device"
        return response

    # -- main loop ----------------------------------------------------------

    def _poll(self) -> None:
        if not self.connected:
            if time.monotonic() >= self._next_connect_at:
                self._connect()
            return

        for line in self._read_lines():
            self._handle(codec.parse_line(line))

        if self._pending_target is not None:
            now = time.monotonic()
            if now - self._last_target_sent_at >= TARGET_MIN_INTERVAL_S:
                if self._write(codec.build_set_joints(self._pending_target)):
                    self._last_target_sent_at = now
                self._pending_target = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._disconnect()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
