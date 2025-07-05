import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class FollowGapF1TENTH(Node):
    def __init__(self):
        super().__init__('follow_gap_f1tenth_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # Parámetros
        self.min_distance = 2.2
        self.window_size = 5
        self.max_steering_angle = 0.4189  # ~24 grados
        self.max_speed = 11.0
        self.min_speed = 0.5
        self.safe_distance = 10.0

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # Reemplazar infinitos por valor máximo
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.clip(ranges, 0, msg.range_max)

        # Suavizado
        smoothed = np.convolve(ranges, np.ones(self.window_size)/self.window_size, mode='same')

        # Eliminar obstáculos cercanos
        cleaned = np.copy(smoothed)
        cleaned[cleaned < self.min_distance] = 0.0

        # Buscar el mayor gap
        gap_start, gap_end = self.find_largest_gap(cleaned)

        if gap_end <= gap_start:
            self.get_logger().warn("No se detectó un gap válido.")
            return

        # Índice del centro del gap
        target_idx = (gap_start + gap_end) // 2
        target_angle = msg.angle_min + target_idx * msg.angle_increment

        # Limitar dirección
        steering_angle = np.clip(target_angle, -self.max_steering_angle, self.max_steering_angle)

        # Velocidad basada en distancia hacia el objetivo
        target_distance = cleaned[target_idx]
        distance_ratio = np.clip(target_distance / self.safe_distance, 0.0, 1.0)

        speed = self.min_speed + (self.max_speed - self.min_speed) * distance_ratio
        speed = np.clip(speed, self.min_speed, self.max_speed)

        self.publish_drive_command(speed, steering_angle)

    def find_largest_gap(self, data):
        max_len = 0
        max_start = 0
        max_end = 0
        current_start = None

        for i in range(len(data)):
            if data[i] > 0:
                if current_start is None:
                    current_start = i
            else:
                if current_start is not None:
                    length = i - current_start
                    if length > max_len:
                        max_len = length
                        max_start = current_start
                        max_end = i - 1
                    current_start = None

        if current_start is not None:
            length = len(data) - current_start
            if length > max_len:
                max_start = current_start
                max_end = len(data) - 1

        return max_start, max_end

    def publish_drive_command(self, speed, steering_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowGapF1TENTH()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
