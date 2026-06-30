import os
import math
import yaml

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory


class SoccerMap(Node):

    def __init__(self):
        super().__init__('soccer_map')

        # ---------- PARAMS ----------
        # Ruta al archivo yaml con la config de la cancha.
        # Por defecto busca "cancha_robocup.yaml" junto a este script,
        # pero se puede sobreescribir con --ros-args -p map_yaml:=/ruta/al/archivo.yaml
        default_yaml = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'cancha_robocup.yaml'
        )
        self.declare_parameter("map_file",os.path.join(get_package_share_directory('config_files'),'maps','cancha_robocup.yaml'))
        yaml_path = self.get_parameter('map_file').value

        self.get_logger().info(f"Cargando configuracion de cancha desde: {yaml_path}")

        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        # ---------- CONFIG DESDE YAML ----------
        self.frame_id = cfg.get('frame_id', 'pumas_map')

        field = cfg.get('field', {})
        self.X_MIN = float(field.get('x_min', -7.0))
        self.X_MAX = float(field.get('x_max', 7.0))
        self.Y_MIN = float(field.get('y_min', -4.48))
        self.Y_MAX = float(field.get('y_max', 4.48))

        # length_m corresponde al eje "largo" (X en el yaml) y width_m al eje "ancho" (Y)
        self.FIELD_LENGTH = float(field.get('length_m', self.X_MAX - self.X_MIN))
        self.FIELD_WIDTH = float(field.get('width_m', self.Y_MAX - self.Y_MIN))

        # landmarks: dict nombre -> {id, points: [[x,y], ...]}
        self.landmarks_cfg = cfg.get('landmarks', {})
        self.landmarks_map = {}
        self.landmarks_ids = {}
        for name, data in self.landmarks_cfg.items():
            pts = data.get('points', []) if isinstance(data, dict) else data
            self.landmarks_map[name] = [(float(x), float(y)) for x, y in pts]
            if isinstance(data, dict) and 'id' in data:
                self.landmarks_ids[name] = data['id']

        # goles (opcional, por si se quiere usar para algo mas adelante)
        self.goals_cfg = cfg.get('goals', {})

        # ---------- ROS ----------
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 1)
        self.tf_pub = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.publish_all)
        self.publish_ground()
        self.publish_field_lines()

    def publish_all(self):
        self.publish_ground()
        #self.publish_field_lines()
        #self.publish_center_circle()
        #self.publish_penalty_lines()
        self.publish_axes()
        self.publish_landmark_tfs()
        self.publish_landmark_markers()
    # -------------------------
    def publish_axes(self):
        axes = Marker()
        axes.header.frame_id = self.frame_id
        axes.ns = "axes"
        axes.id = 999
        axes.type = Marker.LINE_LIST
        axes.scale.x = 0.015
        axes.pose.orientation.w = 1.0

        axes.color.a = 1.0

        # X rojo
        axes.color.r = 1.0
        axes.points.append(self.p(0, 0))
        axes.points.append(self.p(1, 0))

        # Y verde
        axes.color.g = 1.0
        axes.points.append(self.p(0, 0))
        axes.points.append(self.p(0, 1))

        self.marker_pub.publish(axes)

    def p(self, x, y):
        pt = Point()
        pt.x = float(x)
        pt.y = float(y)
        pt.z = 0.01
        return pt

    # -------------------------
    def publish_field_lines(self):
        # En el yaml: x_min/x_max es el eje "largo" del campo (length_m),
        # y_min/y_max es el eje "ancho" (width_m). Mantenemos el dibujo
        # usando directamente los limites del yaml.
        x1, x2 = self.X_MIN, self.X_MAX
        y1, y2 = self.Y_MIN, self.Y_MAX

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "field_lines"
        m.id = 1
        m.type = Marker.LINE_LIST
        m.scale.x = 0.015
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0

        lines = [
            (x1, y1), (x2, y1),
            (x2, y1), (x2, y2),
            (x2, y2), (x1, y2),
            (x1, y2), (x1, y1),
        ]

        for a, b in zip(lines[::2], lines[1::2]):
            m.points.append(self.p(*a))
            m.points.append(self.p(*b))

        self.marker_pub.publish(m)

    def publish_center_circle(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "center_circle"
        m.id = 3
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.015
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0

        R = 0.74
        steps = 60

        for i in range(steps + 1):
            a = 2 * math.pi * i / steps
            m.points.append(self.p(R * math.cos(a), R * math.sin(a)))

        self.marker_pub.publish(m)

    def publish_penalty_lines(self):
            # CORRECCIÓN: El código matemático original asumía que Y era el eje largo.
            # Como X es el eje largo en tu configuración, intercambiamos (x, y) 
            # al momento de crear los puntos para rotar el dibujo 90 grados.
            
            y_old = self.X_MAX - 0.0 
            x1_old, x2_old = 3.04, -3.04

            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "penalty"
            m.id = 4
            m.type = Marker.LINE_LIST
            m.scale.x = 0.015
            m.color.r = m.color.g = m.color.b = 1.0
            m.color.a = 1.0
            m.pose.orientation.w = 1.0

            # Función auxiliar para rotar las coordenadas (intercambiar X e Y)
            def p_swap(x_val, y_val):
                return self.p(y_val, x_val)

            # superior
            m.points.append(p_swap(x1_old + 0.57, y_old))
            m.points.append(p_swap(x2_old - 0.57, y_old))

            m.points.append(p_swap(x1_old + 0.57, -y_old))
            m.points.append(p_swap(x2_old - 0.57, -y_old))

            m.points.append(p_swap(x2_old, y_old))
            m.points.append(p_swap(x2_old, -y_old))
            m.points.append(p_swap(x1_old, y_old))
            m.points.append(p_swap(x1_old, -y_old))

            m.points.append(p_swap(x1_old + 0.57, 0.0))
            m.points.append(p_swap(x2_old - 0.57, 0.0))

            m.points.append(p_swap(x2_old + 0.52, y_old - 1.96))
            m.points.append(p_swap(x1_old - 0.52, y_old - 1.96))

            m.points.append(p_swap(x2_old + 0.52, -y_old + 1.96))
            m.points.append(p_swap(x1_old - 0.52, -y_old + 1.96))

            m.points.append(p_swap(x2_old + 0.52, y_old - 1.96))
            m.points.append(p_swap(x2_old + 0.52, y_old))

            m.points.append(p_swap(x1_old - 0.52, y_old - 1.96))
            m.points.append(p_swap(x1_old - 0.52, y_old))

            m.points.append(p_swap(x1_old - 0.52, -y_old + 1.96))
            m.points.append(p_swap(x1_old - 0.52, -y_old))

            m.points.append(p_swap(x2_old + 0.52, -y_old + 1.96))
            m.points.append(p_swap(x2_old + 0.52, -y_old))

            # inside rectangle
            m.points.append(p_swap(x1_old - 1.52, y_old - 0.97))
            m.points.append(p_swap(x2_old + 1.52, y_old - 0.97))

            m.points.append(p_swap(x1_old - 1.52, -y_old + 0.97))
            m.points.append(p_swap(x2_old + 1.52, -y_old + 0.97))

            m.points.append(p_swap(x1_old - 1.52, y_old - 0.97))
            m.points.append(p_swap(x1_old - 1.52, y_old))

            m.points.append(p_swap(x1_old - 1.52, -y_old + 0.97))
            m.points.append(p_swap(x1_old - 1.52, -y_old))

            m.points.append(p_swap(x2_old + 1.52, y_old - 0.97))
            m.points.append(p_swap(x2_old + 1.52, y_old))

            m.points.append(p_swap(x2_old + 1.52, -y_old + 0.97))
            m.points.append(p_swap(x2_old + 1.52, -y_old))
            
            self.marker_pub.publish(m)

    # -------------------------
    def publish_landmark_tfs(self):
        tfs = []

        for name, positions in self.landmarks_map.items():
            for i, (x, y) in enumerate(positions):
                tf = TransformStamped()
                tf.header.frame_id = self.frame_id
                tf.child_frame_id = f"{name}_{i}"
                tf.header.stamp = self.get_clock().now().to_msg()

                tf.transform.translation.x = x
                tf.transform.translation.y = y
                tf.transform.translation.z = 0.0
                tf.transform.rotation.w = 1.0

                tfs.append(tf)

        self.tf_pub.sendTransform(tfs)

    def publish_ground(self):
        ground = Marker()
        ground.header.frame_id = self.frame_id
        ground.header.stamp = self.get_clock().now().to_msg()
        ground.ns = "ground"
        ground.id = 0
        ground.type = Marker.CUBE
        ground.action = Marker.ADD

        ground.pose.position.x = 0.0
        ground.pose.position.y = 0.0
        ground.pose.position.z = -0.05  # casi cero
        ground.pose.orientation.w = 1.0

        # Igual que antes: scale.x usa el eje "ancho" y scale.y el eje "largo"
        ground.scale.x = self.FIELD_LENGTH
        ground.scale.y = self.FIELD_WIDTH
        ground.scale.z = 0.01

        ground.color.r = 0.1
        ground.color.g = 0.6
        ground.color.b = 0.1
        ground.color.a = 1.0

        self.marker_pub.publish(ground)

    def publish_landmark_markers(self):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "landmarks"
            m.id = 100  # Un solo ID para toda la lista
            m.type = Marker.SPHERE_LIST
            m.action = Marker.ADD

            # Tamaño de las esferas
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15

            # Color (Rojo)
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.pose.orientation.w = 1.0

            # Iterar sobre el diccionario y agregar todos los puntos a la lista
            for name, positions in self.landmarks_map.items():
                for (x, y) in positions:
                    pt = Point()
                    pt.x = float(x)
                    pt.y = float(y)
                    pt.z = 0.05
                    m.points.append(pt)

            self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = SoccerMap()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()