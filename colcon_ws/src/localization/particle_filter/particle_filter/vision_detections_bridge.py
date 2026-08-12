import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from vision_interface.msg import Detections
from localization_msg.msg import VisionLandmark, VisionLandmarkArray

# Static landmark IDs, matching config_files/maps/cancha_robocup.yaml and
# particle_filter/mcl_node.py: 0=ball, 1=goal, 2=robot, 3=L, 4=T, 5=X, 6=center.
# vision_interface class labels come from booster_vision's YoloV8Detector::kClassLabels
# ("Ball", "Goalpost", "Person", "LCross", "TCross", "XCross", "PenaltyPoint",
# "Opponent", "BRMarker"). PenaltyPoint/BRMarker have no matching static
# landmark in the field map, so detections of those classes are dropped.
LABEL_TO_LANDMARK_ID = {
    "Ball": 0,
    "Goalpost": 1,
    "Person": 2,
    "Opponent": 2,
    "LCross": 3,
    "TCross": 4,
    "XCross": 5,
}


class VisionDetectionsBridge(Node):

    def __init__(self):
        super().__init__('vision_detections_bridge')

        # Matches the QoS the booster_vision detector publishes with
        # (reliable, volatile, keep_last depth 1).
        detections_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Detections, '/booster_vision/detection', self.detections_callback, detections_qos)
        self.pub = self.create_publisher(VisionLandmarkArray, '/vision/landmarks', 10)
        self.get_logger().info("Vision detections bridge running...")

    def detections_callback(self, msg):
        landmarks = VisionLandmarkArray()
        landmarks.header = msg.header

        for det in msg.detected_objects:
            landmark_id = LABEL_TO_LANDMARK_ID.get(det.label)
            if landmark_id is None:
                continue

            # `position` is the depth-refined ground-plane estimate (falls back
            # to the monocular `position_projection` when depth wasn't usable);
            # both are [x, y, z] in the robot base_link frame, in metres, with
            # x forward / y left, so bearing = atan2(y, x).
            pos = det.position if len(det.position) >= 2 else det.position_projection
            if len(pos) < 2:
                continue

            landmark = VisionLandmark()
            landmark.id = landmark_id
            landmark.angle = math.atan2(pos[1], pos[0])
            # vision_interface publishes confidence on a 0-100 scale; normalize to 0-1.
            landmark.confidence = float(det.confidence) / 100.0
            landmarks.landmarks.append(landmark)

        if landmarks.landmarks:
            self.pub.publish(landmarks)


def main():
    rclpy.init()
    node = VisionDetectionsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
