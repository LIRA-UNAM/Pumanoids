import rclpy
import time
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pumas_vision_msgs.msg import VisionObject


class GoalkeeperHelper(Node):
    def __init__(self):
        super().__init__('goalkeeper_helper')
        self.bridge = CvBridge()
        self.ball_center_x = 0
        self.ball_center_y = 0
        self.current_position.x = None
        self.current_position.y = None
        self.previous_position.x = None
        self.previous_position.y = None
        self.last_image = None

        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.sub_ball = self.create_subscription(VisionObject, '/vision/ball', self.callback_ball, 1)
        

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_image = cv_image


    def callback_ball(self, msg):
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y
        

    def angle_ball(self):
        #obtener vector del balon
        p_prev = np.array([1, 2]) # Punto inicial (x1, y1)
        p_current = np.array([4, 6]) # Punto final (x2, y2)
        v_ball = p_current - p_prev
        v_x = np.array([1, 0])
        # Calcular el producto escalar
        dot_prod = np.dot(v_ball, v_x)
        # Calcular las normas (magnitudes)
        normas = np.linalg.norm(v_ball) * np.linalg.norm(v_x)
        # Calcular el coseno del ángulo
        cos_theta = dot_prod / normas
        # Asegurar que el valor esté en el rango [-1, 1] para arccos
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        # Calcular el ángulo en radianes y convertir a grados
        angulo_rad = np.arccos(cos_theta)
        return np.degrees(angulo_rad)


    def line_detector(self):
        if self.last_image is None:
            self.get_logger().info("Aún no se ha recibido ninguna imagen")
            return
        
        img = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)
        fld = cv2.ximgproc.createFastLineDetector()
        lines = fld.detect(img)

        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if lines is None:
            self.get_logger().info("No se detectaron líneas")
        else:
            self.get_logger().info("Líneas detectadas:", len(lines))

            count = 0
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]

                    length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                    if length > 50:
                        cv2.line(img_color, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
                        count += 1

        #cv2.imwrite("resultado_lineas_rojas.png", img_color)

    def spin(self): 
        self.line_detector()
        #cv2.imshow('Camara', cv_image)
        #cv2.waitKey(1) 
        time.sleep(0.02) 



def main(args=None):
    rclpy.init(args=args)
    node = GoalkeeperHelper()
    node.spin()
    node.destroy_node()

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()