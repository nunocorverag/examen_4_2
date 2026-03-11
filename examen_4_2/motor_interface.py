#!/usr/bin/env python3
"""
Motor Interface — ROS2 ↔ ESP32 (micro-ROS serial)
───────────────────────────────────────────────────
Lee /joint_states y manda setpoint al ESP32.
Recibe /motor_output del ESP32 y publica feedback.

Topics:
  SUB: /joint_states     (sensor_msgs/JointState)   ← de ROS/Gazebo
  PUB: /set_point        (std_msgs/Float32)          → al ESP32
  SUB: /motor_output     (std_msgs/Float32)          ← del ESP32 [rad/s]
  PUB: /motor_feedback   (sensor_msgs/JointState)    → a los controladores
  PUB: /motor_debug      (std_msgs/String)           → info debug
"""

import rclpy
import numpy as np
import json
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String


class MotorInterface(Node):
    def __init__(self):
        super().__init__('motor_interface')

        # ── Parámetros ────────────────────────────────────────────────
        self.declare_parameter('wheel_name', 'left_wheel_joint')
        self.declare_parameter('omega_max', 9.15)       # rad/s max del motor
        self.declare_parameter('mode', 'velocity')      # 'velocity' o 'position'
        self.declare_parameter('filter_alpha', 0.85)    # filtro EMA

        self.wheel_name  = self.get_parameter('wheel_name').value
        self.omega_max   = self.get_parameter('omega_max').value
        self.mode        = self.get_parameter('mode').value
        self.alpha       = self.get_parameter('filter_alpha').value

        # ── Estado ────────────────────────────────────────────────────
        self.desired_vel = 0.0      # rad/s deseado (de joint_states)
        self.desired_pos = 0.0      # rad deseado (de joint_states)
        self.measured_vel = 0.0     # rad/s medido (del ESP32)
        self.measured_pos = 0.0     # rad acumulado (integrado)
        self.last_time = self.get_clock().now()

        # ── Publishers ────────────────────────────────────────────────
        self.setpoint_pub  = self.create_publisher(Float32,    '/set_point',      10)
        self.feedback_pub  = self.create_publisher(JointState, '/motor_feedback', 10)
        self.debug_pub     = self.create_publisher(String,     '/motor_debug',    10)

        # ── Subscribers ───────────────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)

        self.motor_sub = self.create_subscription(
            Float32, '/motor_output', self._motor_output_cb, 10)

        # ── Timer 50 Hz ───────────────────────────────────────────────
        self.timer = self.create_timer(0.02, self._loop)

        self.get_logger().info(
            f'[MotorInterface] mode={self.mode} | '
            f'wheel={self.wheel_name} | omega_max={self.omega_max} rad/s')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _joint_states_cb(self, msg: JointState):
        """Lee velocidad y posición deseada de joint_states."""
        if self.wheel_name not in msg.name:
            return

        idx = msg.name.index(self.wheel_name)

        if msg.velocity and idx < len(msg.velocity):
            self.desired_vel = msg.velocity[idx]

        if msg.position and idx < len(msg.position):
            self.desired_pos = msg.position[idx]

    def _motor_output_cb(self, msg: Float32):
        """Recibe velocidad medida del ESP32 [rad/s]."""
        raw = msg.data
        # Filtro EMA
        self.measured_vel = (self.alpha * self.measured_vel +
                             (1.0 - self.alpha) * raw)

    # ── Loop principal ────────────────────────────────────────────────

    def _loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Integrar posición desde velocidad medida
        if 0 < dt < 0.5:
            self.measured_pos += self.measured_vel * dt

        # Calcular setpoint normalizado [-1, 1]
        if self.mode == 'velocity':
            sp_norm = np.clip(self.desired_vel / self.omega_max, -1.0, 1.0)
        else:  # position — usar error de posición como setpoint de velocidad
            pos_error = self.desired_pos - self.measured_pos
            sp_norm = np.clip(pos_error / (2.0 * np.pi), -1.0, 1.0)

        # Publicar setpoint al ESP32
        sp_msg = Float32()
        sp_msg.data = float(sp_norm)
        self.setpoint_pub.publish(sp_msg)

        # Publicar feedback a ROS
        fb_msg = JointState()
        fb_msg.header.stamp = now.to_msg()
        fb_msg.name     = [self.wheel_name]
        fb_msg.velocity = [float(self.measured_vel)]
        fb_msg.position = [float(self.measured_pos)]
        self.feedback_pub.publish(fb_msg)

        # Debug
        debug = {
            'mode':        self.mode,
            'desired_vel': round(self.desired_vel, 3),
            'desired_pos': round(self.desired_pos, 3),
            'measured_vel': round(self.measured_vel, 3),
            'measured_pos': round(self.measured_pos, 3),
            'setpoint_norm': round(float(sp_norm), 3),
        }
        self.debug_pub.publish(String(data=json.dumps(debug)))


def main(args=None):
    rclpy.init(args=args)
    node = MotorInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parar motor al salir
        try:
            sp_msg = Float32()
            sp_msg.data = 0.0
            node.setpoint_pub.publish(sp_msg)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
