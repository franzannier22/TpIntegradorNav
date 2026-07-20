#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid

VISITADA = 100      # valor que escribimos en celdas visitadas
NO_VISITADA = -1    # valor inicial (desconocida)


def yaw_de_quaternion(q):
    """Extrae el yaw (rotacion en Z) de un quaternion geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')

        # --- Parametros de la grilla ---
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 400)
        self.declare_parameter('height', 400)
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('mark_radius', 1)
        self.declare_parameter('publish_period', 0.5)   # segundos entre publicaciones

        self.resolution     = self.get_parameter('resolution').value
        self.width           = self.get_parameter('width').value
        self.height          = self.get_parameter('height').value
        self.origin_x        = self.get_parameter('origin_x').value
        self.origin_y        = self.get_parameter('origin_y').value
        self.frame_id        = self.get_parameter('frame_id').value
        self.mark_radius     = self.get_parameter('mark_radius').value
        self.publish_period  = self.get_parameter('publish_period').value

        # --- Estado de la grilla ---
        self.data = [NO_VISITADA] * (self.width * self.height)
        self.visitadas = 0

        # --- Captura de la pose inicial de /odom_real ---
        # /odom_real viene en el frame del MUNDO de Gazebo. /odom (que usa el
        # robot para su TF en RViz) arranca en su propio origen = la pose inicial
        # del robot. Para que las celdas pintadas coincidan con el robot, hay que
        # llevar cada pose del mundo al frame odom: eso es la INVERSA de la pose
        # inicial (restar traslacion inicial + rotar por -yaw0).
        self.origen_capturado = False
        self.x0 = 0.0
        self.y0 = 0.0
        self.yaw0 = 0.0

        # --- Entrada: odometria ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_real', self.odom_cb, 10)

        # --- Salida: grilla de ocupacion ---
        # QoS "latched": durabilidad TRANSIENT_LOCAL para que RViz reciba la
        # ultima grilla aunque se conecte despues de arrancar el nodo.
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.grid_pub = self.create_publisher(OccupancyGrid, '/mapa_visitadas', qos)

        # Timer que publica la grilla periodicamente (no en cada callback).
        self.timer = self.create_timer(self.publish_period, self.publicar_grilla)

        self.get_logger().info(
            f"Grilla {self.width}x{self.height}, res={self.resolution} m/celda, "
            f"origen=({self.origin_x}, {self.origin_y}) en '{self.frame_id}'. "
            f"Publicando en /mapa_visitadas cada {self.publish_period}s."
        )

    def mundo_a_celda(self, x, y):
        """Pose (x, y) YA en frame odom -> (columna i, fila j). None si cae afuera."""
        i = int(math.floor((x - self.origin_x) / self.resolution))
        j = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= i < self.width and 0 <= j < self.height:
            return i, j
        return None

    def marcar(self, i, j):
        """Marca (i, j) y sus vecinas dentro de mark_radius. Devuelve cuantas
        celdas eran nuevas (no estaban visitadas)."""
        nuevas = 0
        r = self.mark_radius
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                ii, jj = i + di, j + dj
                if 0 <= ii < self.width and 0 <= jj < self.height:
                    idx = jj * self.width + ii
                    if self.data[idx] != VISITADA:
                        self.data[idx] = VISITADA
                        nuevas += 1
        return nuevas

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # La primera lectura define el origen del frame odom dentro del mundo.
        if not self.origen_capturado:
            self.x0 = x
            self.y0 = y
            self.yaw0 = yaw_de_quaternion(msg.pose.pose.orientation)
            self.origen_capturado = True
            self.get_logger().info(
                f"Origen /odom_real capturado: ({self.x0:.3f}, {self.y0:.3f}), "
                f"yaw0={math.degrees(self.yaw0):.1f} deg"
            )

        # Transformar pose del MUNDO -> frame odom:
        #   restar traslacion inicial y rotar por -yaw0.
        dx = x - self.x0
        dy = y - self.y0
        cos0 = math.cos(self.yaw0)
        sin0 = math.sin(self.yaw0)
        x_odom =  cos0 * dx + sin0 * dy
        y_odom = -sin0 * dx + cos0 * dy

        celda = self.mundo_a_celda(x_odom, y_odom)
        if celda is None:
            self.get_logger().warn(
                f"Pose odom ({x_odom:.2f}, {y_odom:.2f}) cae fuera de la grilla. "
                f"Revisa width/height/origin.",
                throttle_duration_sec=2.0
            )
            return

        i, j = celda
        nuevas = self.marcar(i, j)
        if nuevas > 0:
            self.visitadas += nuevas

    def publicar_grilla(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)
        # origin es la pose (en frame odom) de la celda (0,0) = esquina inf-izq.
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0   # sin rotacion

        msg.data = self.data   # int8[], row-major: idx = j * width + i

        self.grid_pub.publish(msg)


def main():
    rclpy.init()
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()