from __future__ import annotations

import math
import struct
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import Float64MultiArray

try:
    import serial  # type: ignore
except Exception as exc:  # pragma: no cover
    serial = None
    _SERIAL_IMPORT_ERROR = exc


TX_HEADER = b"\xAA\x55"
RX_HEADER = b"\x55\xAA"

TX_JOINTS = 12
TX_FRAME_LEN = 2 + 1 + (TX_JOINTS * 2) + 2  # hdr + seq + 12*int16 + crc16
RX_FRAME_LEN = 2 + 1 + (9 * 2) + 2 + 1 + 2  # hdr + seq + imu(9xint16) + vbat + fault + crc16


def _crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def _clamp(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


class ServoBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("servo_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("command_topic", "/joint_group_position_controller/commands")
        self.declare_parameter("send_rate_hz", 100.0)
        self.declare_parameter("timeout_s", 0.3)
        self.declare_parameter("idle_mode", "hold_last")  # hold_last | safe_pose

        self.declare_parameter("joint_names", [
            "FL_theta1", "FL_theta2", "FL_theta3",
            "FR_theta1", "FR_theta2", "FR_theta3",
            "RL_theta1", "RL_theta2", "RL_theta3",
            "RR_theta1", "RR_theta2", "RR_theta3",
        ])
        self.declare_parameter("safe_pose_rad", [0.0] * 12)
        self.declare_parameter("max_angle_rad", 1.6)
        self.declare_parameter("angle_scale", 1000.0)  # rad * scale => int16

        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_battery", True)
        self.declare_parameter("publish_fault", True)
        self.declare_parameter("imu_frame_id", "imu_link")

        # IMU scaling (telemetry payload -> SI)
        # accel: g/LSB, gyro: deg/s/LSB, mag: uT/LSB
        self.declare_parameter("imu_accel_g_per_lsb", 0.001)
        self.declare_parameter("imu_gyro_dps_per_lsb", 0.1)
        self.declare_parameter("imu_mag_ut_per_lsb", 0.1)

        if serial is None:
            raise RuntimeError(f"pyserial not available: {_SERIAL_IMPORT_ERROR}")

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.idle_mode = str(self.get_parameter("idle_mode").value).strip().lower()

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != TX_JOINTS:
            raise RuntimeError("joint_names must have length 12")

        self.safe_pose = [float(x) for x in self.get_parameter("safe_pose_rad").value]
        if len(self.safe_pose) != TX_JOINTS:
            raise RuntimeError("safe_pose_rad must have length 12")

        self.max_angle = float(self.get_parameter("max_angle_rad").value)
        self.angle_scale = float(self.get_parameter("angle_scale").value)

        self.publish_imu = bool(self.get_parameter("publish_imu").value)
        self.publish_battery = bool(self.get_parameter("publish_battery").value)
        self.publish_fault = bool(self.get_parameter("publish_fault").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)

        self.accel_g_per_lsb = float(self.get_parameter("imu_accel_g_per_lsb").value)
        self.gyro_dps_per_lsb = float(self.get_parameter("imu_gyro_dps_per_lsb").value)
        self.mag_ut_per_lsb = float(self.get_parameter("imu_mag_ut_per_lsb").value)

        self._last_cmd: List[float] = list(self.safe_pose)
        self._last_cmd_time: float = 0.0
        self._seq: int = 0

        self._serial = serial.Serial(self.port, self.baud, timeout=0.01)
        self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")

        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self._battery_pub = self.create_publisher(Float32, "battery_voltage", 10)
        self._fault_pub = self.create_publisher(UInt8, "servo_fault", 10)

        self._cmd_sub = self.create_subscription(
            Float64MultiArray, self.command_topic, self._on_cmd, 10
        )

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self._send_timer = self.create_timer(1.0 / max(1.0, self.send_rate_hz), self._send_tick)

    def _on_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != TX_JOINTS:
            self.get_logger().warn(f"Ignoring cmd: expected {TX_JOINTS}, got {len(msg.data)}")
            return
        self._last_cmd = [float(x) for x in msg.data]
        self._last_cmd_time = time.time()

    def _select_cmd(self) -> List[float]:
        if self.timeout_s <= 0.0:
            return list(self._last_cmd)
        if (time.time() - self._last_cmd_time) > self.timeout_s:
            if self.idle_mode == "safe_pose":
                return list(self.safe_pose)
        return list(self._last_cmd)

    def _send_tick(self) -> None:
        cmd = self._select_cmd()
        payload = bytearray()
        payload.append(self._seq & 0xFF)
        for a in cmd:
            a = _clamp(a, -self.max_angle, self.max_angle)
            val = int(round(a * self.angle_scale))
            val = max(-32768, min(32767, val))
            payload += struct.pack("<h", val)

        crc = _crc16_ccitt(payload)
        frame = TX_HEADER + payload + struct.pack("<H", crc)
        try:
            self._serial.write(frame)
        except Exception as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
        self._seq = (self._seq + 1) & 0xFF

    def _rx_loop(self) -> None:
        buf = bytearray()
        while rclpy.ok():
            try:
                data = self._serial.read(256)
            except Exception:
                time.sleep(0.01)
                continue
            if data:
                buf.extend(data)
            else:
                time.sleep(0.001)

            # parse frames
            while True:
                idx = buf.find(RX_HEADER)
                if idx < 0:
                    if len(buf) > 1024:
                        buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < RX_FRAME_LEN:
                    break
                frame = bytes(buf[:RX_FRAME_LEN])
                del buf[:RX_FRAME_LEN]
                self._handle_rx_frame(frame)

    def _handle_rx_frame(self, frame: bytes) -> None:
        if not frame.startswith(RX_HEADER):
            return
        payload = frame[2:-2]
        crc_rx = struct.unpack("<H", frame[-2:])[0]
        crc = _crc16_ccitt(payload)
        if crc != crc_rx:
            return

        seq = payload[0]
        imu_raw = struct.unpack("<9h", payload[1:1 + 18])
        vbat_mv = struct.unpack("<h", payload[19:21])[0]
        fault = payload[21]

        if self.publish_imu:
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = self.imu_frame_id

            ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, mx_ut, my_ut, mz_ut = imu_raw
            ax = ax_g * self.accel_g_per_lsb * 9.80665
            ay = ay_g * self.accel_g_per_lsb * 9.80665
            az = az_g * self.accel_g_per_lsb * 9.80665

            gx = math.radians(gx_dps * self.gyro_dps_per_lsb)
            gy = math.radians(gy_dps * self.gyro_dps_per_lsb)
            gz = math.radians(gz_dps * self.gyro_dps_per_lsb)

            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            self._imu_pub.publish(imu)

        if self.publish_battery:
            msg = Float32()
            msg.data = float(vbat_mv) / 1000.0
            self._battery_pub.publish(msg)

        if self.publish_fault:
            msg = UInt8()
            msg.data = fault
            self._fault_pub.publish(msg)

    def destroy_node(self):
        try:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ServoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
