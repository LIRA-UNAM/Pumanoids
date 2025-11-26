import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')
        
        # Initialize CV Bridge for ROS2-OpenCV conversion
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        # Change the topic name to match your camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            1  # Queue size
        )
        
        # Video recording parameters
        self.recording = False
        self.video_writer = None
        self.frame_size = None
        
        # Define the codec and output filename
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')  
        self.output_filename = 'output.avi'
        
        self.get_logger().info('Video recorder node started. Press q to stop recording.')
        
    def image_callback(self, msg):
        
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Initialize video writer on first frame
        if not self.recording:
            self.frame_size = (cv_image.shape[1], cv_image.shape[0])
            self.video_writer = cv2.VideoWriter(
                self.output_filename, 
                self.fourcc, 
                15.0, 
                self.frame_size
            )
            self.recording = True
            self.get_logger().info(f'Started recording to {self.output_filename}')
            
            # Write frame to video file
        self.video_writer.write(cv_image)
            
            # Display the frame
        cv2.imshow('Video Recording', cv_image)
            
            # Check for 'q' key press to stop recording
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop_recording()
                
    
    def stop_recording(self):
        if self.recording:
            if self.video_writer:
                self.video_writer.release()
            cv2.destroyAllWindows()
            self.recording = False
            self.get_logger().info('Video recording stopped and file saved.')
            rclpy.shutdown()
    
    def destroy_node(self):
        self.stop_recording()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    video_recorder_node = VideoRecorderNode()
    
    try:
        rclpy.spin(video_recorder_node)
    except KeyboardInterrupt:
        pass
    finally:
        video_recorder_node.stop_recording()
        video_recorder_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()