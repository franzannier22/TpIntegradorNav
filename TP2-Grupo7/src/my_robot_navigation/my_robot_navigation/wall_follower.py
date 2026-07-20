import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ================== PARAMETROS ==================
DISTANCIA_PARED = 0.18   # distancia deseada a la pared derecha [m]
UMBRAL_FRENTE   = 0.25   # frente bloqueado por debajo de esto [m]
RANGO_PARED     = 0.80   # mas lejos que esto a un lado => NO hay pared [m]

VELOCIDAD = 0.45
VELOCIDAD_MIN_FRAC = 0.30
GIRO      = 1.2
KP        = 2.0
LOOKAHEAD = 0.40

FRENTE_VEL_MAX        = 1.50
ERROR_LATERAL_VEL_MAX = 0.35

THETA     = math.radians(50)
MAX_RANGE = 3.0

# --- Pivote con IMU (giros de 90 EXACTOS) ---
OBJETIVO_PIVOTE = math.radians(90.0)
TOL_PIVOTE      = math.radians(2.0)
GIRO_MIN        = 0.30

# --- Pledge ---
KP_YAW    = 2.0
TOL_SALIR = math.radians(10.0)   # |theta| ~ 0 => rumbo de referencia recuperado
VEL_RECTO_FRAC = 0.80


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.imu_sub  = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        self.meta_sub = self.create_subscription(Bool, '/meta_detectada', self.meta_cb, 10)
        self.meta_alcanzada = False

        # --- IMU / Pledge ---
        self.imu_ok = False
        self.yaw = 0.0
        self.yaw_prev = 0.0
        self.yaw_desenrollado = 0.0   # yaw continuo sin wrap
        self.yaw_ref = 0.0            # referencia GLOBAL fija (rumbo inicial)

        # --- FSM: RECTO | SEGUIR | PIVOTAR ---
        self.estado = 'RECTO'
        self.pivote_inicio = 0.0
        self.pivote_dir = 1.0

        self.get_logger().info("Wall follower iniciado (mano derecha + IMU + Pledge continuo).")

    # ---------------- Callbacks ----------------
    def meta_cb(self, msg: Bool):
        if msg.data and not self.meta_alcanzada:
            self.meta_alcanzada = True
            self.get_logger().info("Meta detectada! Frenando el robot.")

    def imu_cb(self, msg: Imu):
        y = self._yaw(msg.orientation)
        if not self.imu_ok:
            self.yaw_prev = y
            self.yaw_desenrollado = y
            self.yaw_ref = y
            self.imu_ok = True
        self.yaw_desenrollado += self._norm(y - self.yaw_prev)
        self.yaw_prev = y
        self.yaw = y

    # ---------------- Helpers ----------------
    def _yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _norm(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def leer(self, scan, angulo):
        i = int(round((angulo - scan.angle_min) / scan.angle_increment)) % len(scan.ranges)
        r = scan.ranges[i]
        if math.isfinite(r) and r > 0.0:
            return min(r, MAX_RANGE)
        return MAX_RANGE

    def leer_sector_min(self, scan, centro, ancho):
        n = max(1, int(round((ancho / 2.0) / scan.angle_increment)))
        i0 = int(round((centro - scan.angle_min) / scan.angle_increment))
        vals = []
        for k in range(-n, n + 1):
            r = scan.ranges[(i0 + k) % len(scan.ranges)]
            if math.isfinite(r) and r > 0.0:
                vals.append(min(r, MAX_RANGE))
        return min(vals) if vals else MAX_RANGE

    def mapear_0_1(self, valor, lo, hi):
        if hi == lo:
            return 1.0
        t = (valor - lo) / (hi - lo)
        return max(0.0, min(1.0, t))

    # ---------------- Control ----------------
    def scan_cb(self, scan):
        if self.meta_alcanzada:
            self.cmd_pub.publish(Twist())
            return
        if not self.imu_ok:
            self.cmd_pub.publish(Twist())
            return

        frente = self.leer_sector_min(scan, 0.0, math.radians(30))
        der    = self.leer_sector_min(scan, -math.pi / 2.0, math.radians(20))

        b = self.leer(scan, -math.pi / 2.0)
        a = self.leer(scan, -math.pi / 2.0 + THETA)
        alpha     = math.atan2(a * math.cos(THETA) - b, a * math.sin(THETA))
        dist_perp = b * math.cos(alpha)
        dist_proy = dist_perp + LOOKAHEAD * math.sin(alpha)

        # giro neto acumulado respecto a la referencia (Pledge)
        theta = self.yaw_desenrollado - self.yaw_ref

        self.get_logger().info(
            f"[{self.estado}] theta={math.degrees(theta):6.1f} frente={frente:.2f} der={der:.2f}",
            throttle_duration_sec=0.5)

        cmd = Twist()

        # ================= PIVOTAR =================
        if self.estado == 'PIVOTAR':
            recorrido = abs(self.yaw_desenrollado - self.pivote_inicio)
            if recorrido >= OBJETIVO_PIVOTE - TOL_PIVOTE:
                self.estado = 'SEGUIR'
                self.cmd_pub.publish(Twist())
                return
            frac = 1.0 - recorrido / OBJETIVO_PIVOTE
            vel  = max(GIRO_MIN, GIRO * frac)
            cmd.angular.z = self.pivote_dir * vel
            self.cmd_pub.publish(cmd)
            return

        # ================= RECTO =================
        if self.estado == 'RECTO':
            if frente < UMBRAL_FRENTE:
                # CAPTURA: SIEMPRE giro a la izquierda -> la pared queda a la DERECHA.
                # Esto garantiza que theta arranque en +90 y pueda volver a 0 (Pledge).
                self.estado = 'PIVOTAR'
                self.pivote_inicio = self.yaw_desenrollado
                self.pivote_dir = +1.0
                self.cmd_pub.publish(Twist())
                return
            err = self._norm(self.yaw - self.yaw_ref)
            cmd.angular.z = max(-GIRO, min(GIRO, -KP_YAW * err))
            cmd.linear.x  = VELOCIDAD * VEL_RECTO_FRAC
            self.cmd_pub.publish(cmd)
            return

        # ================= SEGUIR (pared derecha) =================
        # 1) frente bloqueado -> pivote de 90 con IMU (mano derecha)
        if frente < UMBRAL_FRENTE:
            self.estado = 'PIVOTAR'
            self.pivote_inicio = self.yaw_desenrollado
            self.pivote_dir = -1.0 if der > RANGO_PARED else +1.0
            self.cmd_pub.publish(Twist())
            return

        # 2) PLEDGE: el giro neto volvio a ~0 (mirando a la referencia) y frente libre
        #    -> suelto la pared y avanzo recto. Sale de la isla en la 1a esquina convexa.
        if abs(theta) < TOL_SALIR and frente > UMBRAL_FRENTE * 1.5:
            self.estado = 'RECTO'
            self.cmd_pub.publish(Twist())
            return

        # 3) seguimiento por geometria (control lateral original, intacto)
        error = DISTANCIA_PARED - dist_proy
        cmd.angular.z = max(-GIRO, min(GIRO, KP * error))
        factor_frente  = self.mapear_0_1(frente, UMBRAL_FRENTE, FRENTE_VEL_MAX)
        factor_lateral = 1.0 - self.mapear_0_1(abs(error), 0.0, ERROR_LATERAL_VEL_MAX)
        factor_giro    = 1.0 - abs(cmd.angular.z) / GIRO
        factor = max(VELOCIDAD_MIN_FRAC, min(factor_frente, factor_lateral, factor_giro))
        cmd.linear.x = VELOCIDAD * factor
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()