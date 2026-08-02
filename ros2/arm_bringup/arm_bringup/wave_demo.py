"""Closed-loop proof that ROS 2 can drive the arm: a slow sine on one joint.

    ros2 run arm_bringup wave_demo
    ros2 run arm_bringup wave_demo --ros-args -p amplitude_deg:=10.0

Enables the arm, sweeps the shoulder around its home position, and e-stops on
Ctrl-C. Deliberately dumb: it knows nothing about joint limits, because the
device rejects anything outside them regardless of what this asks for - if
you set silly parameters you get `out_of_range` errors in the bridge log and
an arm that doesn't move, not an arm that hurts itself.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

try:  # rclpy >= Humble; guarded so an older/newer API can't break startup
    from rclpy.signals import SignalHandlerOptions

    _HAS_SIGNAL_OPTIONS = True
except ImportError:  # pragma: no cover
    _HAS_SIGNAL_OPTIONS = False

SERVICE_WAIT_S = 5.0
CALL_TIMEOUT_S = 3.0


class WaveDemo(Node):
    def __init__(self) -> None:
        super().__init__("wave_demo")

        # Defaults sweep the shoulder 40-80 deg: centred on the bench profile's
        # home (60 deg) and comfortably inside its 0-120 deg limits.
        self.declare_parameter("joint", "shoulder")
        self.declare_parameter("center_deg", 60.0)
        self.declare_parameter("amplitude_deg", 20.0)
        self.declare_parameter("period_s", 6.0)
        # 10 Hz, not the 1-2 Hz the task file suggested: the device restarts an
        # eased move on every set_joints, so sparse updates arrive as a visible
        # step-and-hold rather than the smooth sweep the same task asks for.
        # Stays under arm_bridge's 20 Hz coalescing limit.
        self.declare_parameter("rate_hz", 10.0)

        self._joint = self.get_parameter("joint").get_parameter_value().string_value
        self._center = self.get_parameter("center_deg").get_parameter_value().double_value
        self._amplitude = (
            self.get_parameter("amplitude_deg").get_parameter_value().double_value
        )
        self._period = self.get_parameter("period_s").get_parameter_value().double_value
        rate = self.get_parameter("rate_hz").get_parameter_value().double_value

        self._pub = self.create_publisher(JointState, "/arm/target_joint_states", 10)
        self._enable_client = self.create_client(SetBool, "/arm/enable")
        self._estop_client = self.create_client(Trigger, "/arm/estop")

        self._armed = False
        self._elapsed = 0.0
        self._dt = 1.0 / max(rate, 0.1)
        self.create_timer(self._dt, self._tick)

    # -- lifecycle ----------------------------------------------------------

    def arm(self) -> bool:
        """Enable the arm. Returns False (having moved nothing) if the bridge
        isn't up or the device refuses - e.g. the e-stop pin is open."""
        if not self._enable_client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            self.get_logger().error(
                "/arm/enable unavailable - is arm_bridge running? "
                "(ros2 launch arm_bringup bench.launch.py)"
            )
            return False

        request = SetBool.Request()
        request.data = True
        future = self._enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_S)

        response = future.result()
        if response is None:
            self.get_logger().error("/arm/enable timed out")
            return False
        if not response.success:
            self.get_logger().error(f"device refused enable: {response.message}")
            return False

        self.get_logger().info(
            f"enabled; sweeping '{self._joint}' "
            f"{self._center - self._amplitude:.0f}..{self._center + self._amplitude:.0f} deg "
            f"over {self._period:.1f}s"
        )
        self._armed = True
        return True

    def emergency_stop(self) -> None:
        """Best-effort e-stop on the way out. Never raises: this runs from a
        finally block, and an exception here would mask whatever we were
        already shutting down for."""
        self._armed = False
        if not rclpy.ok():
            self.get_logger().warning(
                "context already shut down; could not send e-stop "
                "(run: ros2 service call /arm/estop std_srvs/srv/Trigger)"
            )
            return
        try:
            if not self._estop_client.service_is_ready():
                return
            future = self._estop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=CALL_TIMEOUT_S)
            self.get_logger().info("e-stopped")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"e-stop on exit failed: {exc}")

    # -- motion -------------------------------------------------------------

    def target_deg(self, elapsed_s: float) -> float:
        return self._center + self._amplitude * math.sin(
            2.0 * math.pi * elapsed_s / self._period
        )

    def _tick(self) -> None:
        if not self._armed:
            return
        self._elapsed += self._dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self._joint]
        msg.position = [math.radians(self.target_deg(self._elapsed))]
        self._pub.publish(msg)


def main(args=None) -> None:
    # Take over SIGINT so Ctrl-C surfaces as KeyboardInterrupt with the context
    # still alive - otherwise rclpy tears it down first and the e-stop below
    # has nothing left to send on.
    if _HAS_SIGNAL_OPTIONS:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    else:  # pragma: no cover
        rclpy.init(args=args)

    node = WaveDemo()
    try:
        if node.arm():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
