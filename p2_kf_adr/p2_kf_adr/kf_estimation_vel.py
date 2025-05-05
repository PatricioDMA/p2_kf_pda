import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import numpy as np

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from .filters.kalman_filter import KalmanFilter_2
from tf_transformations import quaternion_from_euler

class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # TODO: Initialize 6D state and covariance
        initial_state = np.zeros((6, 1)) # [x, y, theta, vx, vy, w]
        initial_covariance = np.eye(6) * 0.1

        self.kf = KalmanFilter_2(initial_state, initial_covariance)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

        self.initial_pose = None
        self.prev_time = None
        self.visualizer = Visualizer()

    def odom_callback(self, msg):
        # TODO: Extract position, orientation, velocities from msg
        if self.initial_pose is None:
            self.initial_pose = odom_to_pose2D(msg)
            self.get_logger().info("Initial pose reference set.")
            return

        current_pose = odom_to_pose2D(msg)
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_pose, current_pose))

        # Time step (dt)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Simulate noisy measurement z
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular

        z = generate_noisy_measurement_2(self.normalized_pose, linear.x, 0.0, angular.z) # z = [x, y, theta, v_x, v_y, omega]

        # TODO: Run predict() and update() of KalmanFilter_2
        mu_pred, Sigma_pred = self.kf.predict(dt)
        mu_updated, Sigma_updated = self.kf.update(z)

        # Log updated state
        self.get_logger().info(f"KF state: {mu_updated}")
        self.get_logger().info(f"Noisy measurement: {z}")

        # TODO: Publish estimated full state
        # Asegurarse de que mu_updated es un tipo adecuado para la visualización
        mu_updated = mu_updated.flatten().astype(float)  # Aplanar el estado
        # Extraer solo la submatriz 3x3 de (x, y, theta)
        Sigma_3x3 = Sigma_updated[:3, :3]
        # Publicar la estimación
        self.visualizer.update(self.normalized_pose.astype(float), mu_updated, Sigma_3x3, step="update")
        self.publish_estimated_pose(mu_updated, Sigma_3x3)
        
        # Extraer el estado estimado para depuración
        x_est = mu_updated[0]
        y_est = mu_updated[1]
        yaw_est = mu_updated[2]
        vx_est = mu_updated[3]
        vy_est = mu_updated[4]
        omega_est = mu_updated[5]

        # Imprimir resultados para depuración
        print(f"[Filtro 2] Estado estimado: x = {x_est:.3f}, y = {y_est:.3f}, yaw = {yaw_est:.3f} rad")
        print(f"[Filtro 2] Velocidades estimadas: vx = {vx_est:.3f}, vy = {vy_est:.3f}, omega = {omega_est:.3f} rad/s")

    def publish_estimated_pose(self, mu, Sigma):
        mu = self.kf.mu.flatten()  # Obtener el estado estimado (3x1) y convertir a 1D
        mu = mu.astype(np.float32)  # Asegurar que mu sea de tipo float32 para evitar errores de tipo
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.pose.position.x = float(mu[0])
        pose_msg.pose.pose.position.y = float(mu[1])
        pose_msg.pose.pose.position.z = 0.0
        
        quat = quaternion_from_euler(0.0, 0.0, float(mu[2]))
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # Asignar una covarianza simple (si no tienes una real)
        # pose_msg.pose.covariance = [
        #     0.1, 0,   0,   0, 0, 0,
        #     0,   0.1, 0,   0, 0, 0,
        #     0,   0,   0,   0.1, 0, 0,
        #     0,   0,   0,   0, 0.1, 0,
        #     0,   0,   0,   0, 0, 0.1
        # ]

        # Solo colocar la submatriz 6x6 de la covarianza correspondiente a x, y, z, rotaciones
        cov = np.zeros((6, 6))
        cov[0, 0] = Sigma[0, 0]  # x
        cov[1, 1] = Sigma[1, 1]  # y
        cov[5, 5] = Sigma[2, 2]  # theta → yaw

        pose_msg.pose.covariance = cov.flatten().tolist()  # Convertir a lista para el mensaje

        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

