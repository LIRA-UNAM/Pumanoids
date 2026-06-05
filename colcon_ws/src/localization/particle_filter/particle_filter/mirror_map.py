from re import X
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math

class SoccerMap(Node):
    def __init__(self):
        super().__init__('soccer_map')
        #----------CONFIG---------------
        self.FIELD_LENGTH = 3.92
        self.FIELD_WIDTH  = 1.97 
        self.landmarks_first_quadrant = {
                "goal":[
                    [0.475,1.96,'T'],
                    [0.475,1.61,'L']
                    ],
                
                "penalty":[
                    [0.785,1.96,'T'],
                    [0.785,1.2,'L'],
                    [0,1.405,'X'],
                    ],
                 "field":[
                    [0.985,1.96,'L']
                    ],
                
                 "center":[
                    [0.25,0,'X'],
                    [0.985,0,'T'],
                    ],
                 } 
       
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 1)
        self.tf_pub = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.publish_all)
    
    def mirror_y(self,point):
        mirror_point = [-point[0], point[1], point[2]]
        return mirror_point

    def mirror_x(self,point):
        mirror_point = [point[0], -point[1], point[2]]
        return mirror_point
    

    def mirror(self,point):
        mirror_point = [-point[0], -point[1], point[2]]
        return mirror_point
    
    def point(self, x, y):
        pt = Point()
        pt.x = float(x)
        pt.y = float(y)
        pt.z = 0.01
        return pt

    #-----------Publishers------------------
    
    def publish_ground(self):
        ground = Marker()
        ground.header.frame_id = "map"
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

    def publish_field_lines(self):
        L = self.FIELD_LENGTH
        W = self.FIELD_WIDTH

        m = Marker()
        m.header.frame_id = "map"
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
            (x1,0)  , (x2,0)  ,
        ]

        for a, b in zip(lines[::2], lines[1::2]):
            m.points.append(self.point(*a))
            m.points.append(self.point(*b))

        self.marker_pub.publish(m)

    def publish_center_circle(self):
    
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "center_circle"
        m.id = 3
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.015
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0

        R = self.landmarks_first_quadrant["center"][0][0] 
        steps = 60

        for i in range(steps + 1):
            a = 2 * math.pi * i / steps
            m.points.append(self.point(R * math.cos(a), R * math.sin(a)))

        self.marker_pub.publish(m)
    def publish_penalty_lines(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "penalty"
        m.id = 4
        m.type = Marker.LINE_LIST
        m.scale.x = 0.015
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0

        #--------GOAL FIRST HALF-------------
        A = self.landmarks_first_quadrant["goal"][0]
        B = self.landmarks_first_quadrant["goal"][1]
        
        A1 = self.mirror_y(A)
        B1 = self.mirror_y(B)

        #--------LEFT---------
        m.points.append(self.point(A[0], A[1])) 
        m.points.append(self.point(B[0], B[1]))
            #--------HORIZONTAL---------
        m.points.append(self.point(B[0],  B[1])) 
        m.points.append(self.point(B1[0], B1[1]))
            #--------RIGHT---------------
        m.points.append(self.point(A1[0], A1[1]))
        m.points.append(self.point(B1[0], B1[1]))
        
        #--------GOAL SECOND HALF-------------
        A = self.mirror_x(A)
        B = self.mirror_x(B)

        A1 = self.mirror_y(A)
        B1 = self.mirror_y(B)

        #--------LEFT---------
        m.points.append(self.point(A[0], A[1])) 
        m.points.append(self.point(B[0], B[1]))
            #--------HORIZONTAL---------
        m.points.append(self.point(B[0],  B[1])) 
        m.points.append(self.point(B1[0], B1[1]))
            #--------RIGHT---------------
        m.points.append(self.point(A1[0], A1[1]))
        m.points.append(self.point(B1[0], B1[1]))
 
        #--------PENALTY FIRST HALF-------------
        A = self.landmarks_first_quadrant["penalty"][0]
        B = self.landmarks_first_quadrant["penalty"][1]
        
        A1 = self.mirror_y(A)
        B1 = self.mirror_y(B)

        #--------LEFT---------
        m.points.append(self.point(A[0], A[1])) 
        m.points.append(self.point(B[0], B[1]))
            #--------HORIZONTAL---------
        m.points.append(self.point(B[0],  B[1])) 
        m.points.append(self.point(B1[0], B1[1]))
            #--------RIGHT---------------
        m.points.append(self.point(A1[0], A1[1]))
        m.points.append(self.point(B1[0], B1[1]))
        
        #--------PENALTY SECOND HALF-------------
        A = self.mirror_x(A)
        B = self.mirror_x(B)

        A1 = self.mirror_y(A)
        B1 = self.mirror_y(B)

        #--------LEFT---------
        m.points.append(self.point(A[0], A[1])) 
        m.points.append(self.point(B[0], B[1]))
            #--------HORIZONTAL---------
        m.points.append(self.point(B[0],  B[1])) 
        m.points.append(self.point(B1[0], B1[1]))
            #--------RIGHT---------------
        m.points.append(self.point(A1[0], A1[1]))
        m.points.append(self.point(B1[0], B1[1]))

        self.marker_pub.publish(m)


    def publish_axes(self):
        axes = Marker()
        axes.header.frame_id = "map"
        axes.ns = "axes"
        axes.id = 999
        axes.type = Marker.LINE_LIST
        axes.scale.x = 0.015
        axes.pose.orientation.w = 1.0

        axes.color.a = 1.0

        # X rojo
        axes.color.r = 1.0
        axes.points.append(self.point(0,0))
        axes.points.append(self.point(1,0))

        # Y verde
        axes.color.g = 1.0
        axes.points.append(self.point(0,0))
        axes.points.append(self.point(0,1))

        self.marker_pub.publish(axes)
   
    def publish_landmark_tfs(self):
        tfs =[]
        landmarks_second_quadrant = {}
        landmarks_third_quadrant  = {}
        landmarks_fourth_quadrant = {}
        for name, points in self.landmarks_first_quadrant.items():
            landmarks_second_quadrant[name] = []
            landmarks_third_quadrant[name] = []
            landmarks_fourth_quadrant[name] = []
            for point in points:
                landmarks_second_quadrant[name].append(self.mirror_y(point))
                landmarks_third_quadrant[name].append(self.mirror(point))
                landmarks_fourth_quadrant[name].append(self.mirror_x(point))
        landmarks = [self.landmarks_first_quadrant, landmarks_second_quadrant, landmarks_third_quadrant, landmarks_fourth_quadrant]
        for i,landmark in enumerate(landmarks):
            for name, points in landmark.items():
                for j, point in enumerate(points):
                    tf = TransformStamped()
                    tf.header.frame_id = "map"
                    tf.child_frame_id = f"{name} {i} quadrant_{point[2]}{j}"
                    tf.header.stamp = self.get_clock().now().to_msg()

                    tf.transform.translation.x = float(point[0])
                    tf.transform.translation.y = float(point[1])
                    tf.transform.translation.z = 0.0 
                    tf.transform.rotation.w = 1.0

                    tfs.append(tf)
        self.tf_pub.sendTransform(tfs)
            

    def publish_all(self):
        self.publish_ground()
        self.publish_field_lines()
        self.publish_center_circle()
        self.publish_penalty_lines()
        self.publish_axes()
        self.publish_landmark_tfs()

    

def main():
    rclpy.init()
    node = SoccerMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

              
