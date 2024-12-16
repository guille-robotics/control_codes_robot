#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class VillelaControl(Node):
    def __init__(self):
        super().__init__('villela_control')
        
        # Parámetros del controlador Villela
        self.Vmax = 0.5           # Velocidad lineal máxima (m/s)
        self.Wmax = math.pi / 4   # Velocidad angular máxima (rad/s)
        self.L = 0.7              # Distancia entre ruedas (no siempre se usa directamente)
        self.Kr_V_RL = 1      # Radio de seguridad (distancia a partir de la cual se escala la velocidad lineal)
        self.target_reached= 0.5 # Parametro en donde v =0.0 y w =0.0 para detener el robot
        
        # Punto objetivo (target)
        self.goal_x = 15.0
        self.goal_y = 12.0

        # Estado del robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Suscriptor a odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publicador a cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Crear un timer para actualizar el control a 10 Hz
        self.timer = self.create_timer(0.1, self.calculate_control)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Convertir de cuaternión a Euler para obtener yaw
        self.orientation_q = msg.pose.pose.orientation
        euler = self.quaternion_to_euler(
            self.orientation_q.x,
            self.orientation_q.y,
            self.orientation_q.z,
            self.orientation_q.w
        )
        self.robot_theta = euler[2]

    def quaternion_to_euler(self, x, y, z, w):
        # Convertir cuaternión a Euler (roll, pitch, yaw)
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
        # Calcular distancia al objetivo
        d = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)

        # Calcular alpha y O_c
        alpha = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
        Oc = math.atan2(math.sin(alpha - self.robot_theta), math.cos(alpha - self.robot_theta))

        # Calcular velocidad angular
        w = self.Wmax * math.sin(Oc)

        # Calcular velocidad lineal
        if d > self.Kr_V_RL:
            v = self.Vmax
        elif d < self.target_reached:
            v = 0.0
            w = 0.0
        else:
            # Escalado lineal hasta Kr_V_RL
            v = d * (self.Vmax / self.Kr_V_RL)

        # Publicar en cmd_vel
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w  # Directamente w en rad/s, sin normalización
        self.cmd_vel_pub.publish(twist)

        # Mensaje informativo
        self.get_logger().info(
            f"d: {d:.4f}, alpha: {alpha:.4f}, Oc: {Oc:.4f}, v: {v:.4f}, w: {w:.4f}, pos: ({self.robot_x:.2f}, {self.robot_y:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = VillelaControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
