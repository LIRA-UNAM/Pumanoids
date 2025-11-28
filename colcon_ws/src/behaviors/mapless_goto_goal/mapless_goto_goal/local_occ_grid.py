#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

import tf2_ros
import tf_transformations
from geometry_msgs.msg import TransformStamped


class LocalOccGrid(Node):
    def __init__(self):
        super().__init__("local_occ_grid")

        self.get_logger().info("INITIALIZING LOCAL OCCUPANCY GRID NODE")

        self.declare_parameter("use_lidar", True)
        self.declare_parameter("use_cloud", False)
        self.declare_parameter("min_x", 0.0)
        self.declare_parameter("max_x", 4.0)
        self.declare_parameter("min_y", -2.0)
        self.declare_parameter("max_y", 2.0)
        self.declare_parameter("min_z", 0.05)
        self.declare_parameter("max_z", 1.5)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("lidar_downsampling", 1)
        self.declare_parameter("cloud_downsampling", 9)
        self.declare_parameter("laser_scan_topic", "/scan")
        self.declare_parameter("point_cloud_topic", "/point_cloud")
        self.declare_parameter("base_link_name", "base_link")

        self.use_lidar = self.get_parameter("use_lidar").value
        self.use_cloud = self.get_parameter("use_cloud").value
        self.min_x = self.get_parameter("min_x").value
        self.max_x = self.get_parameter("max_x").value
        self.min_y = self.get_parameter("min_y").value
        self.max_y = self.get_parameter("max_y").value
        self.min_z = self.get_parameter("min_z").value
        self.max_z = self.get_parameter("max_z").value
        self.resolution = self.get_parameter("resolution").value
        self.lidar_downsampling = self.get_parameter("lidar_downsampling").value
        self.cloud_downsampling = self.get_parameter("cloud_downsampling").value
        self.scan_topic = self.get_parameter("laser_scan_topic").value
        self.cloud_topic = self.get_parameter("point_cloud_topic").value
        self.base_link_name = self.get_parameter("base_link_name").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scan_msg = None
        self.cloud_msg = None

        if self.use_lidar:
            self.create_subscription(LaserScan, self.scan_topic, self.callback_laser_scan, 1)
        if self.use_cloud:
            self.create_subscription(PointCloud2, self.cloud_topic, self.callback_point_cloud, 1)

        self.pub_occ_grid = self.create_publisher(OccupancyGrid, "/local_occ_grid", 30)
        self.pub_occ_grid_array = self.create_publisher(Float32MultiArray, "/local_occ_grid_array", 30)

        self.local_map = OccupancyGrid()
        self.local_map.header.frame_id = self.base_link_name
        self.local_map.info.resolution = self.resolution
        self.local_map.info.width = int((self.max_x - self.min_x) / self.resolution)
        self.local_map.info.height = int((self.max_y - self.min_y) / self.resolution)
        self.local_map.info.origin.position.x = self.min_x
        self.local_map.info.origin.position.y = self.min_y
        self.local_map.info.origin.orientation.w = 1.0
        self.local_map.data = [0] * (self.local_map.info.width * self.local_map.info.height)

        self.local_map_array = Float32MultiArray()
        self.local_map_array.data = [0.0] * len(self.local_map.data)

        self.timer = self.create_timer(1.0/30.0, self.loop)

        self.get_logger().info("LocalOccGrid ready!")
        self.get_logger().info(
            f"LocalOccGrid -> Parameters: "
            f"min_x={self.min_x}  max_x={self.max_x}  "
            f"min_y={self.min_y}  max_y={self.max_y}  "
            f"min_z={self.min_z}  max_z={self.max_z}"
            f"cloud_downsampling={self.cloud_downsampling}  "
            f"lidar_downsampling={self.lidar_downsampling}  "
            f"base link name: {self.base_link_name}"
        )
        self.get_logger().info("Waiting for sensor data...")

    
    def callback_laser_scan(self, msg):
        self.scan_msg = msg

    def callback_point_cloud(self, msg):
        self.cloud_msg = msg

    def get_tf_matrix(self, source_frame):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_link_name, source_frame, rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            translation = tf.transform.translation
            rotation = tf.transform.rotation
            tf_mat = tf_transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
            tf_mat[0, 3] = translation.x
            tf_mat[1, 3] = translation.y
            tf_mat[2, 3] = translation.z
            return tf_mat
        except Exception as e:
            self.get_logger().warn(f"TF unavailable {source_frame} → {self.base_link_name}: {e}")
            return None

    def fill_map_with_cloud(self):
        if self.cloud_msg is None:
            return False

        tf_mat= self.get_tf_matrix(self.cloud_msg.header.frame_id)
        if tf_mat is None:
            return False

        step = self.cloud_downsampling
        points = point_cloud2.read_points(self.cloud_msg, field_names=("x", "y", "z"), skip_nans=True)

        map_width = self.local_map.info.width
        height = self.local_map.info.height
        res = self.resolution
        origin_x = self.min_x
        origin_y = self.min_y

        i = 0
        for p in points:
            if i % step != 0:
                i += 1
                continue
            i += 1

            x, y, z = p
            v = np.array([x, y, z, 1.0])
            v = np.dot(tf_mat, v)

            if not (self.min_x < v[0] < self.max_x and
                    self.min_y < v[1] < self.max_y and
                    self.min_z < v[2] < self.max_z):
                continue

            cell_x = int((v[0] - origin_x) / res)
            cell_y = int((v[1] - origin_y) / res)
            cell = cell_y * map_width + cell_x

            if 0 <= cell < len(self.local_map.data):
                self.local_map.data[cell] = 100

    def fill_map_with_lidar(self):
        if self.scan_msg is None:
            return False

        tf_mat = self.get_tf_matrix(self.scan_msg.header.frame_id)
        if tf_mat is None:
            return False

        map_width = self.local_map.info.width
        height = self.local_map.info.height
        res = self.resolution
        origin_x = self.min_x
        origin_y = self.min_y

        for i in range(0, len(self.scan_msg.ranges), self.lidar_downsampling):
            r = self.scan_msg.ranges[i]
            angle = self.scan_msg.angle_min + i * self.scan_msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            v = np.array([x, y, 0.0, 1.0])
            v = np.dot(tf_mat, v)

            if not (self.min_x < v[0] < self.max_x and
                    self.min_y < v[1] < self.max_y and
                    self.min_z < v[2] < self.max_z):
                continue

            cell_x = int((v[0] - origin_x) / res)
            cell_y = int((v[1] - origin_y) / res)
            cell = cell_y * map_width + cell_x

            if 0 <= cell < len(self.local_map.data):
                self.local_map.data[cell] = 100

    def loop(self):
        # Clear map
        self.local_map.data = [0] * len(self.local_map.data)

        if self.use_lidar:
            self.fill_map_with_lidar()
        if self.use_cloud:
            self.fill_map_with_cloud()

        # Fill array msg
        for i in range(len(self.local_map_array.data)):
            self.local_map_array.data[i] = float(self.local_map.data[i])

        # Publish
        self.local_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_occ_grid.publish(self.local_map)
        self.pub_occ_grid_array.publish(self.local_map_array)


def main(args=None):
    rclpy.init(args=args)
    node = LocalOccGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()