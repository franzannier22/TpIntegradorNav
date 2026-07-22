#!/usr/bin/env python3
"""
camara.py - Nodo PUBLISHER del topico /meta_detectada

Suscribe a /camera/image_raw, busca pixeles rojos (color de la meta)
usando un filtro HSV, y publica el ESTADO ACTUAL en /meta_detectada:
  True  mientras ve suficiente rojo (durante N frames seguidos)
  False apenas deja de ver suficiente rojo

A diferencia de una "confirmacion unica", esto es un ESTADO continuo:
el wall_follower debe frenar SOLO mientras este topico este en True,
y volver a moverse normalmente si vuelve a False (perdio de vista la meta).

Topico:
    /meta_detectada (std_msgs/msg/Bool)  -- PUBLISHER
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2


# ─── Parametros ajustables ────────────────────────────────────────────────────

# Rango HSV para rojo en OpenCV (H: 0-180, S: 0-255, V: 0-255)
# El rojo cruza el limite 0/180 del circulo de matiz, por eso usamos
# DOS rangos (uno cerca de 0, otro cerca de 180) y los combinamos.
ROJO_BAJO_MIN = (0,   90,  60)
ROJO_BAJO_MAX = (10,  255, 255)
ROJO_ALTO_MIN = (170, 90,  60)
ROJO_ALTO_MAX = (180, 255, 255)

# Cuantos pixeles rojos hacen falta para considerar "veo la meta".
# Con camara 320x240 = 76800 pixeles totales.
UMBRAL_PIXELES = 15000

# Anti-parpadeo (debounce): exige N frames SEGUIDOS en el mismo sentido
# antes de cambiar el estado publicado. Sin esto, un solo frame con
# ruido (reflejo, glitch, un pixel rojo perdido) haria que el topico
# parpadee true/false/true todo el tiempo y el robot frene y arranque
# de forma nerviosa. Aplica TANTO para confirmar que aparecio el rojo
# COMO para confirmar que de verdad se perdio.
FRAMES_CONSECUTIVOS_REQUERIDOS = 3

# ────────────────────────────────────────────────────────────────────────────


class DetectorMeta(Node):

    def __init__(self):
        super().__init__('camara')

        self.bridge = CvBridge()

        self.sub_imagen = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)

        self.pub_meta = self.create_publisher(Bool, '/meta_detectada', 10)

        # Estado publicado actualmente (lo que el wall_follower esta viendo)
        self.estado_publicado = False

        # Contador de frames consecutivos que contradicen el estado actual.
        # Ej: si estado_publicado=False y empiezo a ver rojo, este contador
        # sube cada frame con rojo; si llega a FRAMES_CONSECUTIVOS_REQUERIDOS,
        # recien ahi cambio el estado publicado a True (y viceversa).
        self.contador_consecutivo = 0

        self.get_logger().info(
            "Detector de meta (camara) iniciado. "
            f"Buscando rojo, umbral={UMBRAL_PIXELES}px, "
            f"debounce={FRAMES_CONSECUTIVOS_REQUERIDOS} frames."
        )

    def contar_pixeles_rojos(self, imagen_bgr):
        """
        Recibe una imagen en formato BGR (estandar de OpenCV) y devuelve
        la cantidad de pixeles que caen dentro del rango de rojo.
        """
        hsv = cv2.cvtColor(imagen_bgr, cv2.COLOR_BGR2HSV)

        mascara_baja = cv2.inRange(hsv, ROJO_BAJO_MIN, ROJO_BAJO_MAX)
        mascara_alta = cv2.inRange(hsv, ROJO_ALTO_MIN, ROJO_ALTO_MAX)
        mascara_roja = cv2.bitwise_or(mascara_baja, mascara_alta)

        return int(cv2.countNonZero(mascara_roja))

    def image_cb(self, msg: Image):
        try:
            imagen_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"No se pudo convertir la imagen: {e}")
            return

        pixeles_rojos = self.contar_pixeles_rojos(imagen_bgr)
        ve_rojo_este_frame = pixeles_rojos > UMBRAL_PIXELES

        # ¿Este frame coincide con lo que ya tenemos publicado, o lo contradice?
        if ve_rojo_este_frame == self.estado_publicado:
            # Coincide: no hay cambio en curso, resetear el contador
            self.contador_consecutivo = 0
        else:
            # Contradice: podria ser ruido, o el inicio de un cambio real
            self.contador_consecutivo += 1

            if self.contador_consecutivo >= FRAMES_CONSECUTIVOS_REQUERIDOS:
                # Sostenido durante suficientes frames: confirmar el cambio
                self.estado_publicado = ve_rojo_este_frame
                self.contador_consecutivo = 0

                if self.estado_publicado:
                    self.get_logger().info(
                        f"Meta detectada ({pixeles_rojos}px rojos). Frenando.")
                else:
                    self.get_logger().info(
                        "Meta perdida de vista. Reanudando wall following.")

        msg_out = Bool()
        msg_out.data = self.estado_publicado
        self.pub_meta.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    nodo = DetectorMeta()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()