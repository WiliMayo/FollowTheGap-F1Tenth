import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time

class LapTimerNode(Node):
    def __init__(self):
        super().__init__('lap_timer_node')
        self.get_logger().info('¡Nodo iniciado!')
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)

        # Estado del cronómetro
        self.started = False
        self.last_speed = 0.0
        self.start_time = None
        self.lap_number = 0

        # Línea de meta: puntos A y B
        self.line_start = (-1.0, 2.0)
        self.line_end = (1.0, -2.0)

        self.prev_position = None
        self.crossed_line = False

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)

        # Detectar inicio del movimiento
        if not self.started and speed > 0.1:
            self.started = True
            self.start_time = time.time()
            self.get_logger().info('Cronómetro iniciado')

        if not self.started:
            return

        # Obtener posición actual
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_position = (x, y)

        if self.prev_position is not None:
            if self.check_line_cross(self.prev_position, current_position):
                if not self.crossed_line:  # para evitar múltiples detecciones por callback rápido
                    self.lap_number += 1
                    lap_time = time.time() - self.start_time
                    self.get_logger().info(f'Vuelta {self.lap_number}: {lap_time:.2f} segundos')
                    self.start_time = time.time()
                    self.crossed_line = True
            else:
                self.crossed_line = False

        self.prev_position = current_position

    def check_line_cross(self, p1, p2):
        """Verifica si el segmento (p1, p2) cruza la línea de meta"""
        def ccw(a, b, c):
            return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
        a = self.line_start
        b = self.line_end
        return ccw(a, p1, p2) != ccw(b, p1, p2) and ccw(a, b, p1) != ccw(a, b, p2)

def main(args=None):
    rclpy.init(args=args)
    node = LapTimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
