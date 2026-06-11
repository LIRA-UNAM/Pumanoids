import rclpy
import os
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from localization_msg.msg import VisionLandmarkArray
from ament_index_python.packages import get_package_share_directory
import math
import random

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class ParticleFilterNode(Node):

    def __init__(self):
        super().__init__('particle_filter')
        self.forward_speed = 0.4
        self.last_odom_time = None
        self.fov_rad = math.radians(100)
        self.sigma_angle = math.radians(6.0)
        self.pose_pub = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        self.declare_parameter("map_file", os.path.join(get_package_share_directory('config_files'), 'maps', 'cancha_tmr.yaml'))
        map_file = self.get_parameter('map_file').value
        
        self.map_landmarks = {}
        self.read_yaml(map_file)

        self.num_particles = 700
        self.field_x, self.field_y = 6.08, 9.06
        self.particles = []
        self.weights = []
        self.latest_observations = []
        self.last_odom_pose = None
        
        # [NUEVO] Variables para estabilidad y memoria histórica
        self.last_estimated_pose = None 
        self.lost_frames_counter = 0    

        self.particles_pub = self.create_publisher(PoseArray, 'particles', 10)
        self.alphas = [0.02, 0.0002, 0.01, 0.001]
        self.is_moving = False
        self.init_particles()
        
        self.get_logger().info("Particle filter initialized with K-Means & NN Association")

        self.obs_sub = self.create_subscription(
            VisionLandmarkArray, '/vision/landmarks', 
            self.observation_callback, 
            qos_profile_sensor_data)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom_converted',
            self.odom_callback,
            qos_profile_sensor_data)

    def read_yaml(self, config_file):
        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)
            self.map_name = configs["name"]
            self.FIELD_X_MIN = configs["field"]["x_min"] 
            self.FIELD_X_MAX = configs["field"]["x_max"] 
            self.FIELD_Y_MIN = configs["field"]["y_min"] 
            self.FIELD_Y_MAX = configs["field"]["y_max"]
            
            for _, landmark in configs["landmarks"].items():
                self.map_landmarks[landmark["id"]] = []
                for point in landmark["points"]:
                    self.map_landmarks[landmark["id"]].append(tuple(point))

    def init_particles(self):
        self.particles = []
        self.num_particles = 800
        for _ in range(self.num_particles):
            x = random.uniform(-self.field_x/2, self.field_x/2)
            y = random.uniform(-self.field_y/2, self.field_y/2)
            theta = random.uniform(-math.pi, math.pi)
            self.particles.append([x, y, theta])
        self.weights = [1.0 / self.num_particles] * self.num_particles

    # [NUEVO] Algoritmo K-Means ultra ligero nativo (Sin librerías externas)
    def simple_kmeans(self, particles, k=2, max_iters=10):
        if len(particles) < k:
            return [particles], [(0,0)]
        
        # Inicializar centroides aleatorios
        centroids = random.sample([(p[0], p[1]) for p in particles], k)
        clusters = []
        
        for _ in range(max_iters):
            clusters = [[] for _ in range(k)]
            # Asignar al centroide más cercano
            for p in particles:
                dists = [math.hypot(p[0] - c[0], p[1] - c[1]) for c in centroids]
                best_k = dists.index(min(dists))
                clusters[best_k].append(p)
            
            # Recalcular centroides
            new_centroids = []
            for i in range(k):
                if clusters[i]:
                    avg_x = sum(p[0] for p in clusters[i]) / len(clusters[i])
                    avg_y = sum(p[1] for p in clusters[i]) / len(clusters[i])
                    new_centroids.append((avg_x, avg_y))
                else:
                    new_centroids.append(centroids[i])
            centroids = new_centroids
            
        return clusters, centroids

    # [MODIFICADO] Publicación usando K-Means y Memoria Histórica
    def publish_estimated_pose(self):
        if not self.particles:
            return

        # Agrupar las partículas en 2 clusters
        clusters, centroids = self.simple_kmeans(self.particles, k=2)
        best_cluster = self.particles # Default
        
        # Si tenemos memoria de dónde estábamos y detectamos dos nubes
        if self.last_estimated_pose is not None and len(clusters) == 2 and clusters[0] and clusters[1]:
            dist_0 = math.hypot(centroids[0][0] - self.last_estimated_pose[0], centroids[0][1] - self.last_estimated_pose[1])
            dist_1 = math.hypot(centroids[1][0] - self.last_estimated_pose[0], centroids[1][1] - self.last_estimated_pose[1])
            
            # Elegir la nube que requiere el salto más corto desde nuestra última posición
            if dist_0 < dist_1:
                best_cluster = clusters[0]
            else:
                best_cluster = clusters[1]
        else:
            # Si no hay historial o solo hay 1 cluster, elegimos la nube con más partículas
            if len(clusters) == 2:
                best_cluster = clusters[0] if len(clusters[0]) > len(clusters[1]) else clusters[1]
            elif len(clusters) == 1:
                best_cluster = clusters[0]

        # Promediar SOLO el cluster ganador
        avg_x = sum(p[0] for p in best_cluster) / len(best_cluster)
        avg_y = sum(p[1] for p in best_cluster) / len(best_cluster)

        sin_sum = sum(math.sin(p[2]) for p in best_cluster)
        cos_sum = sum(math.cos(p[2]) for p in best_cluster)
        avg_theta = math.atan2(sin_sum, cos_sum)

        # Actualizar memoria histórica
        self.last_estimated_pose = (avg_x, avg_y, avg_theta)

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = avg_x
        msg.pose.position.y = avg_y
        msg.pose.orientation = yaw_to_quaternion(avg_theta)
        self.pose_pub.publish(msg)

    # ----------- MOTION MODEL (Sin cambios) -------------------------
    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_theta = get_yaw_from_quaternion(msg.pose.pose.orientation)
        curr_pose = [curr_x, curr_y, curr_theta]
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
        if self.last_odom_pose is None:
            self.last_odom_pose = curr_pose
            self.last_odom_time = curr_time
            return

        u_t = self.calculate_deltas(curr_pose, self.last_odom_pose)
        delta_rot1, delta_trans_odom, delta_rot2 = u_t
        
        dt = curr_time - self.last_odom_time
        dt = max(dt, 1e-4)
        
        if delta_trans_odom > 0.02:
            delta_trans = self.forward_speed * dt
            self.is_moving = True
        elif abs(delta_rot1 + delta_rot2) > 0.02:
            delta_trans = 0.0
            self.is_moving = True
        else:
            self.is_moving = False
            
        if self.is_moving:
            u_t_corrected = (delta_rot1, delta_trans, delta_rot2)
            new_particles = []
            for p in self.particles:
                new_p = self.sample_motion_model(u_t_corrected, p, self.alphas)
                new_particles.append(new_p)
            self.particles = new_particles
            self.publish_particles()
        
        self.last_odom_pose = curr_pose
        self.last_odom_time = curr_time
        
    def calculate_deltas(self, p_curr, p_prev):
        dx = p_curr[0] - p_prev[0]
        dy = p_curr[1] - p_prev[1]
        delta_trans = math.sqrt(dx**2 + dy**2)
        
        if delta_trans < 0.001:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(math.atan2(dy, dx), p_prev[2])
        delta_rot2 = angle_diff(angle_diff(p_curr[2], p_prev[2]), delta_rot1)
        return delta_rot1, delta_trans, delta_rot2

    def sample_motion_model(self, u_t, x_prev, a):
        dr1, dt, dr2 = u_t
        s1 = math.sqrt(a[0]*dr1**2 + a[1]*dt**2)
        st = math.sqrt(a[2]*dt**2 + a[3]*dr1**2 + a[3]*dr2**2)
        s2 = math.sqrt(a[0]*dr2**2 + a[1]*dt**2)
        
        h_dr1 = dr1 - random.gauss(0, s1)
        h_dt  = dt  - random.gauss(0, st)
        h_dr2 = dr2 - random.gauss(0, s2)
        
        x_new = x_prev[0] + h_dt * math.cos(x_prev[2] + h_dr1)
        y_new = x_prev[1] + h_dt * math.sin(x_prev[2] + h_dr1)
        theta_new = x_prev[2] + h_dr1 + h_dr2
        
        x_new = max(self.FIELD_X_MIN - 0.1, min(x_new, self.FIELD_X_MAX + 0.1))
        y_new = max(self.FIELD_Y_MIN - 0.1, min(y_new, self.FIELD_Y_MAX + 0.1))
        return [x_new, y_new, angle_diff(theta_new, 0)]

#------------OBSERVATION MODEL---------------------
    def observation_callback(self, msg):
        self.latest_observations = sorted(list(msg.landmarks), key=lambda l: l.angle)

        if len(self.latest_observations) < 3:
            # self.get_logger().info(f"Only {len(self.latest_observations)} landmarks — trusting odometry only.")
            self.publish_particles()
            self.publish_estimated_pose()
            return

        if self.latest_observations:
            new_weights = []
            for p in self.particles:
                preds = self.predict_measurements(p)
                weight = self.similarity_function(preds, self.latest_observations)
                new_weights.append(weight)

            avg_weight = sum(new_weights)/len(new_weights)
            max_weight = max(new_weights)
            
            # [MODIFICADO] Lógica de secuestro retardado (evitar pánico prematuro)
            if max_weight < 1e-5:  
                self.lost_frames_counter += 1
                if self.lost_frames_counter > 5:
                    self.get_logger().warn(f"Divergence confirmed. Injecting 10% random particles.")
                    num_random = int(0.1 * self.num_particles)
                    for i in range(num_random):
                        self.particles[i] = [
                            random.uniform(-self.field_x/2, self.field_x/2),
                            random.uniform(-self.field_y/2, self.field_y/2),
                            random.uniform(-math.pi, math.pi)
                        ]
                    self.lost_frames_counter = 0
            else:
                self.lost_frames_counter = 0 # Nos calmamos si vemos algo coherente

            sum_w = sum(new_weights) + 1e-8
            self.weights = [w / sum_w for w in new_weights]
            
            sq_weights = sum([w**2 for w in self.weights])
            neff = 1.0 / sq_weights
            
            if neff < self.num_particles * 0.4: 
                self.resample(robot_is_moving=self.is_moving)
                
            self.publish_particles()
            self.publish_estimated_pose()

    def publish_particles(self):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = 0.0 
            pose.orientation = yaw_to_quaternion(p[2])
            msg.poses.append(pose)
        self.particles_pub.publish(msg)

    def resample(self, robot_is_moving):
        new_particles = []
        M = len(self.particles)
        r = random.uniform(0, 1.0 / M)
        c = self.weights[0]
        i = 0
        
        for m in range(1, M + 1):
            U = r + (m - 1) * (1.0 / M)
            while U > c:
                i = (i + 1) % M
                c += self.weights[i]
                
            p = self.particles[i]
            jitter_xy = 0.01 if not robot_is_moving else 0.0
            jitter_theta = 0.01 if not robot_is_moving else 0.0
            
            nx = p[0] + random.gauss(0, jitter_xy)
            ny = p[1] + random.gauss(0, jitter_xy)
            nt = angle_diff(p[2], random.gauss(0, jitter_theta))
            
            nx = max(self.FIELD_X_MIN, min(nx, self.FIELD_X_MAX))
            ny = max(self.FIELD_Y_MIN, min(ny, self.FIELD_Y_MAX))

            new_particles.append([nx, ny, nt])
            
        self.particles = new_particles
        self.weights = [1.0 / M] * M

    def predict_measurements(self, particle):
        px, py, p_theta = particle
        predicted_dets = []
        for lm_id, positions in self.map_landmarks.items():
            for lm_x, lm_y in positions:
                dx = lm_x - px
                dy = lm_y - py
                abs_angle = math.atan2(dy, dx)
                p_angle = angle_diff(abs_angle, p_theta)
                if abs(p_angle) <= (self.fov_rad / 2.0):
                    predicted_dets.append({
                    "id": lm_id,
                    "angle": p_angle
                })
        return predicted_dets

    # [MODIFICADO] Asociación de Datos robusta por Nearest Neighbor y Gating
    def similarity_function(self, predicted_dets, observations):
        if not observations or not predicted_dets:
            return 1e-7
            
        matched_errors = []
        
        # 1. Búsqueda exhaustiva del Vecino Más Cercano (Nearest Neighbor)
        for obs in observations:
            best_error = None
            for pred in predicted_dets:
                if obs.id == pred['id']:
                    err = angle_diff(obs.angle, pred['angle'])
                    if best_error is None or abs(err) < abs(best_error):
                        best_error = err
            
            # Gating: Si hay un match, validamos que no sea físicamente absurdo (>45 grados)
            if best_error is not None:
                if abs(best_error) < math.radians(45.0):
                    matched_errors.append(best_error)

        if not matched_errors:
            return 1e-8

        num_matches = len(matched_errors)
        num_observations = len(observations)
        num_predictions = len(predicted_dets)
        
        # 2. Gaussian Similarity (Castigo al error)
        similarity = 0.0
        var = self.sigma_angle ** 2
        for err in matched_errors:
            lklihood = -(err**2) / (2*var)
            similarity += lklihood
        
        avg_likelihood = similarity / num_matches
        quality_score = math.exp(avg_likelihood)

        match_success = num_matches / num_observations
        base_weight = quality_score * match_success
        
        obs_not_matched = num_observations - num_matches
        if obs_not_matched > 2:  
            miss_penalty = 0.5 ** (obs_not_matched - 2)
            base_weight *= miss_penalty
            
        pred_not_observed = num_predictions - num_matches
        if pred_not_observed > num_observations:
            excess = pred_not_observed - num_observations
            if excess > 3:  
                false_positive_penalty = 0.3 ** (excess - 3)
                base_weight *= false_positive_penalty
        
        return max(min(base_weight, 1.0), 1e-10)

def main():
    rclpy.init()
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()