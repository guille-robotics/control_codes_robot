#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class IPCControl(Node):
    def __init__(self):
        super().__init__('ipc_control')
        
        # Parámetros del IPC
        self.Vmax = 0.5          # Velocidad lineal máxima (m/s)
        self.Wmax = math.pi / 4  # Velocidad angular máxima (rad/s)
        self.Kr_Prop = 1.0       # Radio de docking (similar a kr)
        self.K1 = 0.1
        self.Kp = 0.75
        self.Ki = 0.00007

        # Objetivo
        self.goal_x = 15.0
        self.goal_y = 12.0

        # Estado del robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Error acumulado para la parte integral del IPC
        self.ErrorAcumulado = 0.0

        # Suscriptor a odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publicador a cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer para control a 10 Hz
        self.timer = self.create_timer(0.1, self.calculate_control)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # Convertir cuaternión a yaw
        orientation_q = msg.pose.pose.orientation
        euler = self.quaternion_to_euler(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        self.robot_theta = euler[2]

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 =  1.0 if t2 >  1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def calculate_control(self):
        # Calcular distancia y Oc
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        d = math.sqrt(dx*dx + dy*dy)

        alpha = math.atan2(dy, dx)
        Oc = math.atan2(math.sin(alpha - self.robot_theta), math.cos(alpha - self.robot_theta))

        # Calcular factor p
        error = Oc
        p = ((math.pi - abs(error))/math.pi)

        # Velocidad lineal
        v = min(self.K1 * d * p, self.Vmax)
        if p > 0.9 and d > self.Kr_Prop:
            v = self.Vmax
        if d < self.Kr_Prop:
            v = 0.0

        # Parte integral en el error angular
        self.ErrorAcumulado += error

        # Velocidad angular
        # Nota: En el código original no se limita 'w' a Wmax, pero podrías hacerlo si deseas.
        w = self.Kp * math.sin(Oc) + self.Ki * self.ErrorAcumulado

        if d < self.Kr_Prop:
            # Si ya estamos muy cerca, paramos
            w = 0.0

        # Opcional: Limitar w a ±Wmax
        if w > self.Wmax:
            w = self.Wmax
        elif w < -self.Wmax:
            w = -self.Wmax

        # Publicar velocidades
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(
            f"d: {d:.4f}, Oc: {Oc:.4f}, p: {p:.4f}, v: {v:.4f}, w: {w:.4f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IPCControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

