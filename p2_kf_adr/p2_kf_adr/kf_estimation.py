import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.msg import WheelVels

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 
from geometry_msgs.msg import PoseWithCovarianceStamped 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # TODO: Initialize filter with initial state and covariance
        initial_state = np.zeros((3, 1)) # [x, y, theta]'
        initial_covariance = np.eye(3) * 0.1

        self.kf = KalmanFilter(initial_state, initial_covariance)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

        self.prev_time = None
        self.initial_pose = None

    def odom_callback(self, msg):
        # TODO: Extract velocities and timestep
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        # Ver instante actual y calcular dt
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return
        dt = current_time - self.prev_time
        self.prev_time = current_time

        u = np.array([v, w])  # señal de control

        # TODO: Run predict() and update() of KalmanFilter
        self.kf.predict(u, dt)

        z = odom_to_pose2D(msg)  # posición observada (convertida)
        self.kf.update(z)

        # TODO: Publish estimated state
        mu = self.kf.mu.flatten()  # Obtener el estado estimado (3x1) y convertir a 1D
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = float(mu[0])
        pose_msg.pose.pose.position.y = float(mu[1])
        pose_msg.pose.pose.position.z = 0.0

        # Convertir a cuaternión para publicar
        quat = self.euler_to_quaternion(0, 0, mu[2])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # Opcional: rellenar la matriz de covarianza en el mensaje si se desea
        self.publisher.publish(pose_msg)

        # Imprimir resultados para depuración
        print(f"[KalmanFilter_1 Estimate] x: {mu[0]:.3f}, y: {mu[1]:.3f}, theta: {mu[2]:.3f} rad")
        
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Conversión euler->cuaternión
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z
            cr * cp * cy + sr * sp * sy   # w
        ]
        return q

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
