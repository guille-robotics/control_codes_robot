#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class VillelaSimpleObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('villela_simple_avoidance')
        
        # Parámetros de Villela
        self.Vmax = 0.5
        self.Wmax = math.pi/4
        self.Kr_V_RL = 1.0    # Radio para escalado lineal
        self.Kr_Prop = 0.5    # Distancia para considerar objetivo alcanzado

        # Parámetro para evitar obstáculos
        self.noDetectionDistance = 1  # Si un obstáculo está a menos de 0.5 m, actuar
        self.avoid_turn_speed = math.pi/8  # velocidad angular para esquivar
        
        # Objetivo
        self.goal_x = 15.0
        self.goal_y = 12.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.laser_ranges = []

        # Suscripciones
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publicador
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer para control a 10 Hz
        self.timer = self.create_timer(0.1, self.calculate_control)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.robot_theta = yaw

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w*x + y*z)
        t1 = +1.0 - 2.0*(x*x + y*y)
        roll_x = math.atan2(t0, t1)

        t2 = 2.0*(w*y - z*x)
        t2 = max(min(t2,1.0),-1.0)
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w*z + x*y)
        t4 = +1.0 - 2.0*(y*y + z*z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def calculate_control(self):
        if not self.laser_ranges:
            return

        # Calcular distancia y Oc al objetivo
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        d = math.sqrt(dx*dx + dy*dy)
        alpha = math.atan2(dy, dx)
        Oc = math.atan2(math.sin(alpha - self.robot_theta), math.cos(alpha - self.robot_theta))

        # Control Villela básico
        w = self.Wmax * math.sin(Oc)
        if d > self.Kr_V_RL:
            v = self.Vmax
        else:
            v = d*(self.Vmax/self.Kr_V_RL)

        if d < 0.5:
            v = 0.0
            w = 0.0

        if d < self.Kr_Prop:
            v = 0.0
            w = 0.0

        # Detectar el obstáculo más cercano
        min_dist = min(self.laser_ranges) if self.laser_ranges else self.noDetectionDistance*2
        if math.isinf(min_dist) or math.isnan(min_dist):
            min_dist = self.noDetectionDistance*2

        # Si hay un obstáculo cerca
        if min_dist < self.noDetectionDistance:
            # Reducir la velocidad lineal
            v = 0.0
            # Aplicar una ligera rotación para esquivar.
            # Puedes mejorar esta lógica (p.e. rotar hacia el lado con más espacio),
            # pero por sencillez, rotamos a la derecha.
            w = self.avoid_turn_speed

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(f"d: {d:.2f}, Oc: {Oc:.2f}, v: {v:.2f}, w: {w:.2f}, min_dist: {min_dist:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = VillelaSimpleObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
