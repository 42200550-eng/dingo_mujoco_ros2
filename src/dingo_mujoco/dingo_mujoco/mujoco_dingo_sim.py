from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def _clamp(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


def _parse_float_list(value: Any, *, expected_len: int, name: str) -> List[float]:
    # Accept either a real list/tuple (YAML) or a string (CSV / Python-list-like).
    if isinstance(value, (list, tuple)):
        out = [float(x) for x in value]
    elif isinstance(value, str):
        s = value.strip()
        if s.startswith('[') and s.endswith(']'):
            s = s[1:-1].strip()
        # Split by comma or whitespace
        parts = [p for p in (x.strip() for x in s.replace(',', ' ').split()) if p]
        out = [float(p) for p in parts]
    else:
        raise ValueError(f"{name} must be a list/tuple or string, got {type(value)}")

    if len(out) != expected_len:
        raise ValueError(f"{name} length must be {expected_len}, got {len(out)}")
    return out


class MujocoDingoSim(Node):
    """MuJoCo plant for Dingo that keeps CHAMP/ROS2 wiring.

    Subscribes:
      - /joint_group_position_controller/commands (Float64MultiArray, 12)

    Publishes:
      - /joint_states (sensor_msgs/JointState)
      - /clock (rosgraph_msgs/Clock) for use_sim_time
      - /odom (nav_msgs/Odometry) + TF odom->base_link

    Notes:
    - This node is intended to replace Gazebo Classic for faster, smoother contact.
    - Actuation is position-servo-like: MuJoCo <position> actuators expect ctrl = q_des.
    - We keep a per-joint sign map to match URDF/Gazebo conventions if desired.
    """

    def __init__(self) -> None:
        super().__init__('mujoco_dingo_sim')

        # Model
        self.declare_parameter('model_xml_path', '')
        # ROS 2 may auto-declare use_sim_time (e.g., via launch). Avoid double declaration.
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass

        # ROS topics
        self.declare_parameter('cmd_topic', '/joint_group_position_controller/commands')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        # Sim loop
        self.declare_parameter('sim_rate_hz', 500.0)
        self.declare_parameter('publish_rate_hz', 200.0)
        # /clock can be published at a lower rate than sim stepping to reduce DDS load
        self.declare_parameter('clock_publish_rate_hz', 200.0)

        # Rendering (optional). MuJoCo python does not show a window unless you launch a viewer.
        # Keep default headless for performance and for machines without a display.
        self.declare_parameter('render', False)
        self.declare_parameter('render_rate_hz', 60.0)

        # Joint ordering MUST match CHAMP + your joints_map
        self.declare_parameter('joints', [
            'FL_theta1', 'FL_theta2', 'FL_theta3',
            'FR_theta1', 'FR_theta2', 'FR_theta3',
            'RL_theta1', 'RL_theta2', 'RL_theta3',
            'RR_theta1', 'RR_theta2', 'RR_theta3',
        ])

        # Optional sign compensation (same idea as dingo_effort_bridge)
        self.declare_parameter('joint_signs', [
            1.0, -1.0, -1.0,
            -1.0, -1.0, -1.0,
            1.0, -1.0, -1.0,
            -1.0, -1.0, -1.0,
        ])
        # CSV override for easy paste from CLI/launch args (string type)
        self.declare_parameter('joint_signs_csv', '')

        # What to do when command is missing
        self.declare_parameter('idle_mode', 'hold_initial')  # hold_initial | hold_last
        self.declare_parameter('initial_positions', [0.0, 0.8, -0.18] * 4)
        # CSV override for easy paste from CLI/launch args (string type)
        self.declare_parameter('initial_positions_csv', '')

        # Safety / smoothing for position targets
        self.declare_parameter('max_target_rate_rad_s', 8.0)  # rate limit on q_des

        # Control source:
        # - 'ros'    : write actuator ctrl from CHAMP commands (default)
        # - 'viewer' : do NOT write actuator ctrl; lets MuJoCo viewer sliders drive ctrl
        self.declare_parameter('control_mode', 'ros')

        self.model_xml_path = str(self.get_parameter('model_xml_path').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)

        self.sim_rate_hz = float(self.get_parameter('sim_rate_hz').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.clock_publish_rate_hz = float(self.get_parameter('clock_publish_rate_hz').value)

        self.render = bool(self.get_parameter('render').value)
        self.render_rate_hz = float(self.get_parameter('render_rate_hz').value)

        self.joints: List[str] = list(self.get_parameter('joints').value)
        joint_signs_csv = str(self.get_parameter('joint_signs_csv').value).strip()
        if joint_signs_csv:
            self.joint_signs = _parse_float_list(
                joint_signs_csv,
                expected_len=len(self.joints),
                name='joint_signs_csv',
            )
        else:
            self.joint_signs = _parse_float_list(
                self.get_parameter('joint_signs').value,
                expected_len=len(self.joints),
                name='joint_signs',
            )
        if len(self.joint_signs) != len(self.joints):
            raise ValueError('joint_signs length must match joints')

        self.idle_mode = str(self.get_parameter('idle_mode').value).strip().lower()
        if self.idle_mode not in ('hold_initial', 'hold_last'):
            raise ValueError("idle_mode must be 'hold_initial' or 'hold_last'")

        initial_positions_csv = str(self.get_parameter('initial_positions_csv').value).strip()
        if initial_positions_csv:
            self.initial_positions = _parse_float_list(
                initial_positions_csv,
                expected_len=len(self.joints),
                name='initial_positions_csv',
            )
        else:
            self.initial_positions = _parse_float_list(
                self.get_parameter('initial_positions').value,
                expected_len=len(self.joints),
                name='initial_positions',
            )

        self.max_target_rate = float(self.get_parameter('max_target_rate_rad_s').value)
        self.control_mode = str(self.get_parameter('control_mode').value).strip().lower()
        if self.control_mode not in ('ros', 'viewer'):
            raise ValueError("control_mode must be 'ros' or 'viewer'")

        # ROS IO
        self._cmd_sub = self.create_subscription(Float64MultiArray, self.cmd_topic, self._on_cmd, 10)
        self._js_pub = self.create_publisher(JointState, self.joint_states_topic, 20)

        # /clock should be easy to consume by many subscribers and CLI tools.
        # Use RELIABLE so ROS2 CLI defaults (e.g., `ros2 topic hz`) can receive without
        # requiring QoS overrides, but keep VOLATILE to avoid stale latched samples
        # causing time-jump warnings after restarts.
        clock_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._clock_pub = self.create_publisher(Clock, '/clock', clock_qos)
        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._tf_br = TransformBroadcaster(self)

        # Utilities
        self._dump_pose_srv = self.create_service(Trigger, '~/dump_pose', self._on_dump_pose)

        # Internal state
        self._last_cmd: Optional[List[float]] = None
        self._last_cmd_wall_t: Optional[float] = None
        self._target_q: List[float] = list(self.initial_positions)

        # Load MuJoCo
        self._mj = None
        self._model = None
        self._data = None
        self._act_ids: List[int] = []
        self._joint_qpos_adr: Dict[str, int] = {}
        self._joint_dof_adr: Dict[str, int] = {}

        self._load_mujoco_or_die()

        # Optional viewer
        self._viewer = None
        self._next_render_wall: Optional[float] = None
        if self.render:
            self._start_viewer_best_effort()

        # Timers
        self._sim_dt = 1.0 / max(50.0, self.sim_rate_hz)
        self._pub_dt = 1.0 / max(10.0, self.publish_rate_hz)
        self._clock_dt = 1.0 / max(1.0, self.clock_publish_rate_hz)
        self._next_pub_wall = time.time()
        self._next_clock_wall = time.time()
        if self.render:
            self._next_render_wall = time.time()

        self.get_logger().info(
            f"MuJoCo sim ready: sim_rate={self.sim_rate_hz}Hz, pub_rate={self.publish_rate_hz}Hz, cmd={self.cmd_topic}, control_mode={self.control_mode}"
        )

        # IMPORTANT: Run sim loop on wall-clock (steady time).
        # If this node uses use_sim_time=true, ROS-time timers won't tick until /clock is received,
        # but /clock is produced by this node. Using a steady clock breaks that circular dependency.
        self._wall_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self._sim_timer = self.create_timer(self._sim_dt, self._step, clock=self._wall_clock)

        # Prime /clock immediately so late joiners and tooling can get a message even before motion.
        self._publish_clock()

    def _load_mujoco_or_die(self) -> None:
        try:
            import mujoco  # type: ignore
            self._mj = mujoco
        except Exception as e:
            self.get_logger().error(
                'Python package "mujoco" not found. Install it first, e.g.:\n'
                '  pip install mujoco\n'
                'Then rebuild/overlay if needed and re-run this node.'
            )
            raise

        if not self.model_xml_path:
            raise ValueError('model_xml_path parameter is required')

        self._model = self._mj.MjModel.from_xml_path(self.model_xml_path)
        self._data = self._mj.MjData(self._model)

        # Map joints (hinge) and actuators
        for jn in self.joints:
            jid = self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                raise RuntimeError(f'Joint not found in MJCF: {jn}')
            qpos_adr = int(self._model.jnt_qposadr[jid])
            dof_adr = int(self._model.jnt_dofadr[jid])
            self._joint_qpos_adr[jn] = qpos_adr
            self._joint_dof_adr[jn] = dof_adr

            aid = self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_ACTUATOR, jn)
            if aid < 0:
                raise RuntimeError(
                    f'Actuator not found for joint {jn}. Expected actuator name == joint name.'
                )
            self._act_ids.append(int(aid))

        # Initialize joint pose
        for i, jn in enumerate(self.joints):
            self._data.qpos[self._joint_qpos_adr[jn]] = float(self.initial_positions[i])
        self._mj.mj_forward(self._model, self._data)

    def _start_viewer_best_effort(self) -> None:
        # Viewer requires a working OpenGL context (GLFW). If unavailable, keep running headless.
        try:
            import mujoco.viewer  # type: ignore

            self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
            self.get_logger().info('MuJoCo viewer window opened (render=true).')
        except Exception as e:
            self._viewer = None
            self.get_logger().error(
                'render=true but failed to open MuJoCo viewer. '\
                'This usually means no DISPLAY / OpenGL / GLFW available. '\
                f'Error: {e}'
            )

    def _on_cmd(self, msg: Float64MultiArray) -> None:
        if self.control_mode == 'viewer':
            # In viewer mode, ignore ROS commands so sliders remain authoritative.
            return
        if len(msg.data) != len(self.joints):
            self.get_logger().warn(f'Ignoring cmd: expected {len(self.joints)} values, got {len(msg.data)}')
            return
        self._last_cmd = [float(x) for x in msg.data]
        self._last_cmd_wall_t = time.time()

    def _on_dump_pose(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # Dump current joint qpos values in the node's joint order.
        # This is intended for copying into launch initial_positions.
        pairs = []
        vals: List[float] = []
        for jn in self.joints:
            q = float(self._data.qpos[self._joint_qpos_adr[jn]])
            vals.append(q)
            pairs.append(f'{jn}: {q:.6f}')

        response.success = True
        response.message = (
            'initial_positions: [' + ', '.join(f'{v:.6f}' for v in vals) + ']\n'
            + 'initial_positions_csv: ' + ','.join(f'{v:.6f}' for v in vals) + '\n'
            + 'joint_positions:\n  ' + '\n  '.join(pairs)
        )
        return response

    def _get_desired_positions(self) -> List[float]:
        if self._last_cmd is None:
            if self.idle_mode == 'hold_initial':
                return list(self.initial_positions)
            return list(self._target_q)
        return list(self._last_cmd)

    def _apply_rate_limit(self, q_des: List[float], dt: float) -> None:
        if self.max_target_rate <= 0.0:
            self._target_q = q_des
            return
        max_step = self.max_target_rate * dt
        out: List[float] = []
        for i in range(len(self.joints)):
            cur = float(self._target_q[i])
            nxt = float(q_des[i])
            out.append(cur + _clamp(nxt - cur, -max_step, max_step))
        self._target_q = out

    def _step(self) -> None:
        # 1) Determine desired joint positions
        q_des = self._get_desired_positions()
        self._apply_rate_limit(q_des, self._sim_dt)

        # 2) Write MuJoCo actuator controls
        # MuJoCo <position> actuator uses ctrl = desired position.
        if self.control_mode != 'viewer':
            for i, act_id in enumerate(self._act_ids):
                sign = 1.0 if float(self.joint_signs[i]) >= 0.0 else -1.0
                self._data.ctrl[act_id] = sign * float(self._target_q[i])

        # 3) Step physics
        self._mj.mj_step(self._model, self._data)

        # Publish /clock (rate-limited) so use_sim_time subscribers and ros2 tools keep receiving time.
        now_wall = time.time()
        if now_wall >= self._next_clock_wall:
            self._next_clock_wall = now_wall + self._clock_dt
            self._publish_clock()

        # 3.5) Render (best effort)
        if self._viewer is not None and self._next_render_wall is not None:
            now = time.time()
            if now >= self._next_render_wall:
                self._next_render_wall = now + (1.0 / max(1.0, self.render_rate_hz))
                try:
                    self._viewer.sync()
                except Exception:
                    # If the window is closed or context lost, stop rendering.
                    self._viewer = None

        # 4) Publish at lower rate
        now = time.time()
        if now >= self._next_pub_wall:
            self._next_pub_wall = now + self._pub_dt
            self._publish_ros()

    def _publish_clock(self) -> None:
        # MuJoCo time is self._data.time (seconds)
        t = float(self._data.time)
        sec = int(math.floor(t))
        nsec = int((t - float(sec)) * 1e9)
        clk = Clock()
        clk.clock.sec = sec
        clk.clock.nanosec = nsec
        self._clock_pub.publish(clk)

    def _sim_time_msg(self) -> TimeMsg:
        t = float(self._data.time)
        sec = int(math.floor(t))
        nsec = int((t - float(sec)) * 1e9)
        msg = TimeMsg()
        msg.sec = sec
        msg.nanosec = nsec
        return msg

    def _publish_ros(self) -> None:
        stamp = self._sim_time_msg()

        # joint states
        js = JointState()
        js.header.stamp = stamp
        js.name = list(self.joints)
        js.position = []
        js.velocity = []
        js.effort = []

        for jn in self.joints:
            q = float(self._data.qpos[self._joint_qpos_adr[jn]])
            dq = float(self._data.qvel[self._joint_dof_adr[jn]])
            js.position.append(q)
            js.velocity.append(dq)
        self._js_pub.publish(js)

        # odom + TF: use freejoint root if present
        # If the model has a freejoint at root, the first 7 qpos are base pose.
        # We publish odom->base_link.
        if int(self._model.nq) >= 7 and int(self._model.nv) >= 6:
            x = float(self._data.qpos[0])
            y = float(self._data.qpos[1])
            z = float(self._data.qpos[2])
            # MuJoCo quaternion order in qpos is (w, x, y, z)
            qw = float(self._data.qpos[3])
            qx = float(self._data.qpos[4])
            qy = float(self._data.qpos[5])
            qz = float(self._data.qpos[6])

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = z
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            self._odom_pub.publish(odom)

            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf_br.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = MujocoDingoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if getattr(node, '_viewer', None) is not None:
                node._viewer.close()
        except Exception:
            pass
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Ignore double-shutdown edge cases during launch SIGINT handling.
            pass
