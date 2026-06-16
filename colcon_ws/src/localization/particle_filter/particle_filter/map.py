import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math

class SoccerMap(Node):

    def __init__(self):
        super().__init__('soccer_map')

        # ---------- CONFIG ----------
        self.FIELD_LENGTH = 10.86
        self.FIELD_WIDTH  = 7.22

        self.landmarks = {
            "goal": [
                (0.52,4.53),
                (-0.52, 4.53),
                (-0.52,-4.53),
                (0.52,-4.53),
                    ], 
            "L": [
                (-2.52, 2.57),
                (-2.52, -2.57),
                ( 2.52, 2.57),
                ( 2.52, -2.57),

                (-1.52, 3.56),
                ( 1.52,  3.56),
                ( -1.52, -3.56),
                ( 1.52, -3.56),
		        ],

            "T": [
                (3.61, 4.53),
                (-3.61, 4.53),
                (3.61, -4.53),
                (-3.61, -4.53),

                (3.04, 4.53),
                (-3.04, 4.53),
                (-3.04,  -4.53),
                (3.04, -4.53),

                (2.52, 4.53),
                (-2.52, 4.53),
                (2.52, -4.53),
                (-2.52, -4.53),

                (1.52, 4.53),
                (-1.52, 4.53),
                (1.52, -4.53),
                (-1.52, -4.53),
                ],
            "X": [
                (0.74, 0.0),
                (-0.74, 0.0),
                (0.0, 0.0),
                (3.04, 0.0),
                (-3.04,  0.0),
                (0.0, -3.06),
                (0.0, 3.06),
                ],
            "center": [(0.0,0.0)],    
        }
        self.landmarks_map = {}

        for name, positions in self.landmarks.items():
            self.landmarks_map[name] = []
            for (x, y) in positions:
                self.landmarks_map[name].append(
                    (x, y)
                )

        # ---------- ROS ----------
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 1)
        self.tf_pub = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.publish_all)
        self.publish_ground()
        self.publish_field_lines()

    def publish_all(self):
        self.publish_ground()
        self.publish_field_lines()
        self.publish_center_circle()
        self.publish_penalty_lines()
        self.publish_axes()
        self.publish_landmark_tfs()

    # -------------------------
    def publish_axes(self):
        axes = Marker()
        axes.header.frame_id = "pumas_map"
        axes.ns = "axes"
        axes.id = 999
        axes.type = Marker.LINE_LIST
        axes.scale.x = 0.015
        axes.pose.orientation.w = 1.0

        axes.color.a = 1.0

        # X rojo
        axes.color.r = 1.0
        axes.points.append(self.p(0,0))
        axes.points.append(self.p(1,0))

        # Y verde
        axes.color.g = 1.0
        axes.points.append(self.p(0,0))
        axes.points.append(self.p(0,1))

        self.marker_pub.publish(axes)

    def p(self, x, y):
        pt = Point()
        pt.x = float(x)
        pt.y = float(y)
        pt.z = 0.01
        return pt


    # -------------------------
    def publish_field_lines(self):
        L = self.FIELD_LENGTH
        W = self.FIELD_WIDTH

        m = Marker()
        m.header.frame_id = "pumas_map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "field_lines"
        m.id = 1
        m.type = Marker.LINE_LIST
        m.scale.x = 0.015
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 2.0
        m.pose.orientation.w = 1.0

        x1, x2 = -W/2, W/2
        y1, y2 = -L/2, L/2

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
        m.header.frame_id = "pumas_map"
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
        y = self.FIELD_LENGTH/2 - 0.9
        x1, x2 = 3.04, -3.04

        m = Marker()
        m.header.frame_id = "pumas_map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "penalty"
        m.id = 4
        m.type = Marker.LINE_LIST
        m.scale.x = 0.015
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0

        # superior
        m.points.append(self.p(x1+0.57, y))
        m.points.append(self.p(x2-0.57,  y))

        m.points.append(self.p(x1+0.57, -y))
        m.points.append(self.p(x2-0.57,  -y))

        m.points.append(self.p(x2, y))
        m.points.append(self.p(x2, -y))
        m.points.append(self.p(x1, y))
        m.points.append(self.p(x1, -y))

        m.points.append(self.p(x1+0.57, 0.0))
        m.points.append(self.p(x2-0.57,0.0))

        m.points.append(self.p(x2+0.52, y-1.96))
        m.points.append(self.p(x1-0.52, y-1.96))

        m.points.append(self.p(x2+0.52, -y+1.96))
        m.points.append(self.p(x1-0.52, -y+1.96))

        m.points.append(self.p(x2+0.52, y-1.96))
        m.points.append(self.p(x2+0.52, y))

        m.points.append(self.p(x1-0.52, y-1.96))
        m.points.append(self.p(x1-0.52, y))

        m.points.append(self.p(x1-0.52, -y+1.96))
        m.points.append(self.p(x1-0.52, -y))

        m.points.append(self.p(x2+0.52, -y+1.96))
        m.points.append(self.p(x2+0.52, -y))

        # inside rectangle
        m.points.append(self.p(x1-1.52, y - 0.97))
        m.points.append(self.p(x2+1.52, y - 0.97))

        m.points.append(self.p(x1-1.52, -y + 0.97))
        m.points.append(self.p(x2+1.52, -y + 0.97))

        m.points.append(self.p(x1-1.52, y - 0.97))
        m.points.append(self.p(x1-1.52, y))

        m.points.append(self.p(x1-1.52, -y +0.97))
        m.points.append(self.p(x1-1.52, -y))

        m.points.append(self.p(x2+ 1.52, y -0.97))
        m.points.append(self.p(x2+1.52, y))

        m.points.append(self.p(x2+1.52, -y + 0.97))
        m.points.append(self.p(x2+1.52, -y))
        self.marker_pub.publish(m)
    # -------------------------
    def publish_landmark_tfs(self):
        tfs = []

        for name, positions in self.landmarks_map.items():
            for i, (x, y) in enumerate(positions):
                tf = TransformStamped()
                tf.header.frame_id = "pumas_map"
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
        ground.header.frame_id = "pumas_map"
        ground.header.stamp = self.get_clock().now().to_msg()
        ground.ns = "ground"
        ground.id = 0
        ground.type = Marker.CUBE
        ground.action = Marker.ADD

        ground.pose.position.x = 0.0
        ground.pose.position.y = 0.0
        ground.pose.position.z = -0.05   # casi cero
        ground.pose.orientation.w = 1.0

        ground.scale.x = self.FIELD_WIDTH
        ground.scale.y = self.FIELD_LENGTH
        ground.scale.z = 0.01

        ground.color.r = 0.1
        ground.color.g = 0.6
        ground.color.b = 0.1
        ground.color.a = 1.0 

        self.marker_pub.publish(ground)

        
    def publish_landmark_markers(self):
        i = 0
        for name, positions in self.landmarks_map.items():
            for (x, y) in positions:
                m = Marker()
                m.header.frame_id = "pumas_map"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "landmarks"
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD

                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0

                m.scale.x = m.scale.y = m.scale.z = 0.12
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 2.5

                self.marker_pub.publish(m)
                i += 1




def main():
    rclpy.init()
    node = SoccerMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

