import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from localization_msg.msg import VisionLandmarkArray
from nav_msgs.msg import Odometry
import numpy as np
import math
import random
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

FIELD_X_MIN = -0.8
FIELD_X_MAX =  0.8
FIELD_Y_MIN = -1.3
FIELD_Y_MAX =  1.3

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q
# [-pi , pi]
def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))
def get_yaw_from_quaternion (q):
    """ Converts quaternion to yaw [rad] """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
# ----------------------------------------
class ParticleFilterNode(Node):

    def __init__(self):
        super().__init__('particle_filter')
        #FOV
        self.fov_rad = math.radians(53.7)
        self.sigma_angle = math.radians(5.0)
        # 0:ball 1:goal, 2:robot, 3:L, 4:T, 5:X 
        self.map_landmarks = {
            1: [(0.5, 1.4),
                (-0.23, 1.4),
                        ], 
            3: [(-0.8, 1.3),
                (-0.8, -1.3),
                ( 0.8, 1.3),
                ( 0.8, -1.3),

                (-0.5, 0.9),
                ( 0.5,  0.9),
                ( -0.5, -0.9),
                ( 0.5, -0.9),

                ( -0.23, -1.15),
                ( 0.23, -1.15),
                ( 0.23, 1.15),
                ( -0.23, 1.15),], 
            4: [(-0.5, 1.3),
                (0.5, 1.3),
                (0.5, -1.3),
                (-0.5, -1.3),

                (-0.23,  1.3),
                (0.23, 1.3),
                (-0.23,  -1.3),
                (0.23, -1.3),

                (-0.8, 0.0),
                (0.8, 0.0),],
            5: [(-0.2, 0.9),
                (0.2, 0.9),
                (-0.2, -0.9),
                (0.2, -0.9),
                (-0.21,  0.0),
                (0.21, 0.0),] 
        }
# --- GRAPH Neff and Average weight ---
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.log_file = open('pf_data_log.csv', 'w')
        self.log_file.write('time,avg_weight,neff,max_weight,moving\n')
        self.get_logger().info("Archivo de log pf_data_log.csv creado.")
#------------------------------------------------------
        # Particles
        self.num_particles = 500
        self.field_x, self.field_y = 1.6, 2.6
        self.particles = []
        self.weights = []
        self.latest_observations = []
        self.last_odom_pose = None
        self.particles_pub = self.create_publisher(PoseArray, 'particles', 10)
        #Noise values for motion algorithm
        '''
        alpha1: Noise in rotation caused by rotation.
        alpha2: Noise in rotation caused by translation. 
        alpha3: Noise in translation caused by translation.
        alpha4: Noise in translation caused by rotation. 
        '''
        self.alphas = [0.0001, 0.001, 0.0035, 0.001]
        # Initialization
        self.is_moving = False
        self.init_particles()
        # self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info("Particle filter")

        #  ROS Sub/Pub
        self.obs_sub = self.create_subscription(
            VisionLandmarkArray, '/vision/landmarks', 
            self.observation_callback, 
            qos_profile_sensor_data)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback,
            qos_profile_sensor_data)

    def init_particles(self):
        self.particles = []
        for _ in range(self.num_particles):
            x = random.uniform(-self.field_x/2, self.field_x/2)
            y = random.uniform(-self.field_y/2, self.field_y/2)
            theta = random.uniform(-math.pi, math.pi)
            self.particles.append([x, y, theta])
        self.weights = [1.0 / self.num_particles] * self.num_particles
    def __del__(self):
        if hasattr(self, 'log_file'):
            self.log_file.close()
            self.get_logger().info("Archivo de log cerrado correctamente.")
# -----------MOTION MODEL -------------------------
# Based on Table 5.6 page 136 PR
    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_theta = get_yaw_from_quaternion(msg.pose.pose.orientation)
        curr_pose = [curr_x, curr_y, curr_theta]

        if self.last_odom_pose is None:
            self.last_odom_pose = curr_pose
            return

        # 1. Calculate Deltas (Algorithm Table 5.6 lines 2-4)
        u_t = self.calculate_deltas(curr_pose, self.last_odom_pose)
        
        # 2. Update particles only if there is significant motion
        if u_t[1] > 0.02 or abs(u_t[0] + u_t[2]) > 0.02:
            self.is_moving = True
            new_particles = []
            for p in self.particles:
                # Algorithm 5.4 lines 5-11
                new_p = self.sample_motion_model(u_t, p, self.alphas)
                new_particles.append(new_p)
            self.particles = new_particles
            # Movement
            self.publish_particles()

        self.last_odom_pose = curr_pose
    #Lines 2-4
    def calculate_deltas(self, p_curr, p_prev):
        dx = p_curr[0] - p_prev[0]
        dy = p_curr[1] - p_prev[1]
        #δtrans​=sqrt[(xˉ−xˉ′)2+(yˉ​−yˉ​′)2​]
        delta_trans = math.sqrt(dx**2 + dy**2)
        
        if delta_trans < 0.001:
            delta_rot1 = 0.0
        else:
            #δrot1​=atan2(yˉ​′−yˉ​,xˉ′−xˉ)−θˉ
            delta_rot1 = angle_diff(math.atan2(dy, dx), p_prev[2])
        # δrot2​=θˉ′−θˉ−δrot1​
        delta_rot2 = angle_diff(angle_diff(p_curr[2], p_prev[2]), delta_rot1)
        return delta_rot1, delta_trans, delta_rot2

    def sample_motion_model(self, u_t, x_prev, a):
        dr1, dt, dr2 = u_t
        # Standard Deviation
        s1 = math.sqrt(a[0]*dr1**2 + a[1]*dt**2)
        st = math.sqrt(a[2]*dt**2 + a[3]*dr1**2 + a[3]*dr2**2)
        s2 = math.sqrt(a[0]*dr2**2 + a[1]*dt**2)
        
        # Sample with noise
        h_dr1 = dr1 - random.gauss(0, s1)
        h_dt  = dt  - random.gauss(0, st)
        h_dr2 = dr2 - random.gauss(0, s2)
        
        # New pose
        x_new = x_prev[0] + h_dt * math.cos(x_prev[2] + h_dr1)
        y_new = x_prev[1] + h_dt * math.sin(x_prev[2] + h_dr1)
        theta_new = x_prev[2] + h_dr1 + h_dr2
        # CLAMPING: No permitir que salgan de los límites
        # Usamos un pequeño margen (0.1) por si los landmarks están fuera
        x_new = max(FIELD_X_MIN - 0.1, min(x_new, FIELD_X_MAX + 0.1))
        y_new = max(FIELD_Y_MIN - 0.1, min(y_new, FIELD_Y_MAX + 0.1))
        return [x_new, y_new, angle_diff(theta_new, 0)]

#------------OBSERVATION MODEL---------------------
    def observation_callback(self, msg):
        self.latest_observations = sorted(list(msg.landmarks), key=lambda l: l.angle)

        if self.latest_observations:
            #  CALCULAR PESOS (Update Step)
            new_weights = []
            for p in self.particles:
                preds = self.predict_measurements(p)
                # Función de similitud como la probabilidad p(y|x)
                weight = self.similarity_function(preds, self.latest_observations)
                new_weights.append(weight)

            #Check average of weights
            avg_weight = sum (new_weights)/len(new_weights)
            max_weight = max (new_weights)
            # DETECCIÓN DE DIVERGENCIA SEVERA
            if max_weight < 1e-3:  # Pesos muy bajos
                self.get_logger().warn(
                    f"Divergence detected - Max weight: {max_weight:.6f}"
                )
                # Inyectar % de partículas aleatorias
                num_random = int(0.2 * self.num_particles)
                for i in range(num_random):
                    self.particles[i] = [
                        random.uniform(-self.field_x/2, self.field_x/2),
                        random.uniform(-self.field_y/2, self.field_y/2),
                        random.uniform(-math.pi, math.pi)
                    ]
                self.get_logger().info(f"Looking for position...")
            #  NORMALIZAR PESOS (Total Probability Theorem) 
            sum_w = sum(new_weights) + 1e-9
            self.weights = [w / sum_w for w in new_weights]
            #----LOGS----
            self.get_logger().info(f"Average weight : {avg_weight:.4f} | Max weight : {max_weight:.4f}")

            # Neff = 1 / sum(w^2). Indica qué tan bien repartida está la probabilidad.
            sq_weights = sum([w**2 for w in self.weights])
            neff = 1.0 / sq_weights
            self.get_logger().info(f"Neff : {neff:.4}")
            # ----------------------------------
            if neff < self.num_particles * 0.5: 
                self.resample(robot_is_moving=self.is_moving)
                self.get_logger().info("Resampling Executed.")
            #  RESAMPLING (Selección Natural)
            #  Visualizar particulas
            self.publish_particles()
        else:
            return

    def publish_particles(self):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Iterar sobre el enjambre real de partículas
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = 0.0 
            
            # yaw (p[2]) a cuaternión para ROS 2
            pose.orientation = yaw_to_quaternion(p[2])
            msg.poses.append(pose)

        self.particles_pub.publish(msg)
        
        # Log para saber qué tan dispersas están
        #self.get_logger().info(f"Publicadas {len(self.particles)} partículas.")

    def resample(self, robot_is_moving):
        #Table 4.4 pag 110 PR Algorithm Low_variance_sampler(Xt , Wt ):
        new_particles = []
        M = len(self.particles)
        
        # LÍNEA 3: r = rand(0, M^-1)
        r = random.uniform(0, 1.0 / M)
        
        # LÍNEA 4: c = w[0] (el peso de la primer partícula)
        c = self.weights[0]
        
        # LÍNEA 5: i = 1 
        i = 0
        
        # LÍNEA 6: for m = 1 to M
        for m in range(1, M + 1):
            # SAMPLING: Usamos lógica de Low Variance
            # LÍNEA 7: U = r + (m - 1) * M^-1
            U = r + (m - 1) * (1.0 / M)
            
            # LÍNEA 8-11: while U > c
            while U > c:
                i = (i + 1) % M
                c += self.weights[i]
                
            # LÍNEA 12: add x[i] to X_new
            p = self.particles[i]
            if not robot_is_moving:
                jitter_xy = 0.0002
                jitter_theta = 0.002
            else:
                jitter_xy = 0.0
                jitter_theta = 0.0
            
            nx = p[0] + random.gauss(0, jitter_xy)
            ny = p[1] + random.gauss(0, jitter_xy)
            nt = angle_diff(p[2], random.gauss(0, jitter_theta))
            # --- CLAMPING FINAL (No más partículas fuera de la cancha) ---
            nx = max(FIELD_X_MIN, min(nx, FIELD_X_MAX))
            ny = max(FIELD_Y_MIN, min(ny, FIELD_Y_MAX))

            new_particles.append([nx, ny, nt])
        self.particles = new_particles
        # Reiniciamos pesos para el siguiente ciclo
        self.weights = [1.0 / M] * M

    def predict_measurements(self, particle):
        px, py, p_theta = particle
        predicted_dets = []
        for lm_id, positions in self.map_landmarks.items():
            for lm_x, lm_y in positions:
                dx = lm_x - px
                dy = lm_y - py
                abs_angle = math.atan2(dy,dx)
                p_angle = angle_diff(abs_angle,p_theta)
                if abs(p_angle) <= (self.fov_rad / 2.0):
                    predicted_dets.append({
                    "id": lm_id,
                    "angle": p_angle
                })
        predicted_dets.sort(key=lambda d: d["angle"])
        return predicted_dets

    def similarity_function(self, predicted_dets, observations):
        if not observations or not predicted_dets:
            return 1e-8
        matched_errors = []
        matched_pred_indices = set()
        obs_idx = 0
        pred_idx = 0

        while obs_idx < len(observations) and pred_idx < len(predicted_dets):
            obs = observations[obs_idx] #instancia de VisionLandmark
            pred = predicted_dets[pred_idx]
            if obs.id == pred['id']:
                #ID match
                error = angle_diff(obs.angle, pred['angle'])
                matched_errors.append(error)
                matched_pred_indices.add(pred_idx)
                obs_idx += 1
                pred_idx += 1
            else :
                #only predd pointer (avoid negative false detections)
                pred_idx += 1
        # bad particle
        if not matched_errors:
            return 1e-9

        num_matches = len(matched_errors)
        num_observations = len(observations)
        num_predictions = len(predicted_dets)
        #------ Gaussian Similarity--------
        # error ~ 0 ; p_weight ~ 1
        similarity = 0.0
        # variance = sigma**2
        var = self.sigma_angle ** 2
        for err in matched_errors:
            #Gaussian function pag. 155 PR
            lklihood = -(err**2)/(2*var)
            #p. 152 ec 6.2 p(zt | xt , m)=KPIk=1 p(zkt | xt , m)
            similarity += lklihood
        
        avg_likelihood = similarity / num_matches
        quality_score = math.exp(avg_likelihood)

        # 2. NUEVO: Ratio de éxito basado en MATCHES sobre OBSERVACIONES
        # Esto favorece explicar lo que vemos, no penaliza lo que no vemos
        match_success = num_matches / num_observations
        
        # 3. Peso base = calidad × éxito en explicar observaciones
        base_weight = quality_score * match_success
        
        # 4. PENALIZACIÓN MÁS SUAVE por observaciones no explicadas
        # Toleramos que 1-2 observaciones no tengan predicción
        obs_not_matched = num_observations - num_matches
        
        if obs_not_matched > 2:  # Tolerancia de 2 landmarks
            # Penalización moderada (no exponencial)
            miss_penalty = 0.5 ** (obs_not_matched - 2)
            base_weight *= miss_penalty
        pred_not_observed = num_predictions - num_matches
        # Tolerancia: Es OK predecir más landmarks de los que ves
        # (el detector puede fallar en detectar algunos)
        if pred_not_observed > num_observations:
            # Solo penalizar si hay DEMASIADAS predicciones extra
            excess = pred_not_observed - num_observations
            if excess > 2.5:  # Tolerancia de 3 landmarks extra
                false_positive_penalty = 0.3 ** (excess - 3)
                base_weight *= false_positive_penalty
        
        return max(min(base_weight, 1.0), 1e-10)
            # pag. 16 PR
        # return math.exp(similarity)

def main():
    rclpy.init()
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()   
