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
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

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
        self.current_odom_pose = [0.0, 0.0, 0.0]
        self.forward_speed = 0.5
        self.last_odom_time = None
        self.fov_rad = math.radians(95) #FOV
        self.sigma_angle = math.radians(4.0)
        self.pose_pub = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        self.declare_parameter("map_file",os.path.join(get_package_share_directory('config_files'),'maps','cancha_robocup.yaml'))
        map_file = self.get_parameter('map_file').value
        # 0:ball 1:goal, 2:robot, 3:L, 4:T, 5:X
        self.map_landmarks ={}
        self.read_yaml(map_file)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.best_x     = 0.0
        self.best_y     = 0.0
        self.best_theta = 0.0
        self.curr_pose = [0.0,0.0,0.0] 
        #TF2 variables
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for TF 10 Hz
        self.tf_timer = self.create_timer(0.1, self.publish_tf_continuous)
#------------------------------------------------------
        # Particles
        self.field_x, self.field_y = 6.08, 9.06
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
        self.alphas = [0.0002, 0.0002, 0.001, 0.0001]

        # ---- KALMAN FUSION: predicción (movimiento) vs medición (visión) ----
        # kf_epsilon evita divisiones entre cero cuando una varianza da 0.0
        # min_variance evita que el filtro se vuelva "sobreconfiado" (K saturado)
        # cuando por casualidad todas las partículas quedan idénticas.
        self.kf_epsilon = 1e-6
        self.min_variance_xy = 1e-4          # m^2
        self.min_variance_theta = math.radians(1.0) ** 2  # rad^2
        self.kf_gain = (0.0, 0.0, 0.0)       # último (Kx, Ky, Ktheta), útil para debug/logs
        self.kf_var  = (0.0, 0.0, 0.0)       # última varianza fusionada por eje

        # Initialization
        self.is_moving = False
        self.init_particles()
        # self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info("Particle filter")

        #  ROS Sub/Pub
        self.tf_timer = self.create_timer(0.15, self.tf_callback)
        self.obs_sub = self.create_subscription(
            VisionLandmarkArray, '/vision/landmarks', 
            self.observation_callback, 
            qos_profile_sensor_data)

    # -----------KALMAN FUSION (predicción por movimiento vs medición por visión)-----
    def weighted_circular_mean(self, angles, weights):
        """Media circular ponderada de una lista de ángulos [-pi, pi]."""
        sin_sum = sum(w * math.sin(a) for a, w in zip(angles, weights))
        cos_sum = sum(w * math.cos(a) for a, w in zip(angles, weights))
        return math.atan2(sin_sum, cos_sum)

    def weighted_mean_and_var(self, particles, weights):
        """
        Calcula la media ponderada (x, y, theta) y la varianza ponderada
        de la nube de partículas.

        Se usa dos veces por ciclo:
          - Con pesos UNIFORMES -> representa la "predicción" (solo lo que
            dice el modelo de movimiento/odometría, sin info de visión).
          - Con self.weights (los pesos que acaba de calcular
            similarity_function) -> representa la "medición" (lo que ve
            el robot ahora mismo).
        """
        total_w = sum(weights)
        if total_w <= 0:
            total_w = self.kf_epsilon

        mean_x = sum(p[0] * w for p, w in zip(particles, weights)) / total_w
        mean_y = sum(p[1] * w for p, w in zip(particles, weights)) / total_w
        mean_theta = self.weighted_circular_mean([p[2] for p in particles], weights)

        var_x = sum(w * (p[0] - mean_x) ** 2 for p, w in zip(particles, weights)) / total_w
        var_y = sum(w * (p[1] - mean_y) ** 2 for p, w in zip(particles, weights)) / total_w
        var_theta = sum(w * angle_diff(p[2], mean_theta) ** 2 for p, w in zip(particles, weights)) / total_w

        # Piso de varianza: nunca dejar que una distribución "colapsada"
        # (var=0) haga que el Kalman ignore por completo a la otra fuente.
        var_x = max(var_x, self.min_variance_xy)
        var_y = max(var_y, self.min_variance_xy)
        var_theta = max(var_theta, self.min_variance_theta)

        return mean_x, mean_y, mean_theta, var_x, var_y, var_theta

    def kalman_fuse_linear(self, pred_mean, pred_var, meas_mean, meas_var):
        """
        Fusión Kalman clásica de dos gaussianas independientes (para x o y).
        K cercano a 1 -> se confía en la medición (visión).
        K cercano a 0 -> se confía en la predicción (movimiento).
        """
        k = pred_var / (pred_var + meas_var + self.kf_epsilon)
        fused_mean = pred_mean + k * (meas_mean - pred_mean)
        fused_var  = (1.0 - k) * pred_var
        return fused_mean, fused_var, k

    def kalman_fuse_angle(self, pred_mean, pred_var, meas_mean, meas_var):
        """Igual que kalman_fuse_linear pero respetando el wrap-around de theta."""
        innovation = angle_diff(meas_mean, pred_mean)
        k = pred_var / (pred_var + meas_var + self.kf_epsilon)
        fused_mean = angle_diff(pred_mean + k * innovation, 0.0)
        fused_var  = (1.0 - k) * pred_var
        return fused_mean, fused_var, k

    def compute_best_pose(self):
        """
        Fusión Kalman entre:
          - "Predicción": estimación de las partículas con pesos UNIFORMES,
            o sea, solo lo que dice el modelo de movimiento/odometría.
          - "Medición": estimación de las partículas con self.weights,
            es decir, lo que "ve" el robot según el modelo de observación.

        El Kalman decide en automático a cuál confiar más, según qué tan
        dispersa (incierta) esté cada una de las dos nubes ponderadas.
        Reemplaza el umbral fijo (max_w > uniform_w * 1.5) que había antes.
        """
        if not self.particles:
            return
        if not self.weights or len(self.weights) != len(self.particles):
            # Sin pesos válidos de visión todavía -> solo promedio de movimiento
            self.weights = [1.0 / len(self.particles)] * len(self.particles)

        n = len(self.particles)
        uniform_weights = [1.0 / n] * n

        # 1) Predicción -> solo movimiento (ignora el peso de visión)
        pred_x, pred_y, pred_theta, pred_var_x, pred_var_y, pred_var_theta = \
            self.weighted_mean_and_var(self.particles, uniform_weights)

        # 2) Medición -> lo que "ve" el robot (pesos actualizados por visión)
        meas_x, meas_y, meas_theta, meas_var_x, meas_var_y, meas_var_theta = \
            self.weighted_mean_and_var(self.particles, self.weights)

        # 3) Fusión Kalman eje por eje
        self.best_x, var_x, kx = self.kalman_fuse_linear(pred_x, pred_var_x, meas_x, meas_var_x)
        self.best_y, var_y, ky = self.kalman_fuse_linear(pred_y, pred_var_y, meas_y, meas_var_y)
        self.best_theta, var_theta, kth = self.kalman_fuse_angle(pred_theta, pred_var_theta, meas_theta, meas_var_theta)

        self.kf_gain = (kx, ky, kth)
        self.kf_var  = (var_x, var_y, var_theta)

    def publish_estimated_pose(self):
        """Publica PoseStamped de la pose estimada en pumas_map."""
        msg = PoseStamped()
        msg.header.frame_id = "pumas_map"
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x    = self.best_x
        msg.pose.position.y    = self.best_y
        msg.pose.orientation.z = float(math.sin(self.best_theta / 2.0))
        msg.pose.orientation.w = float(math.cos(self.best_theta / 2.0))
        self.pose_pub.publish(msg)

    def publish_tf_continuous(self):
        """
        Calcula y publica TF pumas_map → pumas_odom.
        pumas_map → pumas_odom → pumas_base_link
        odometría ya publica odom → base_link con la pose
        (ox, oy, o_theta). El filtro (best_x, best_y, best_theta) 
        en pumas_map
        Queremos:
            T_map_odom  ⊕  T_odom_base  =  T_map_base 

            map_x     =  best_x - ox*cos(o_theta) + oy*sin(o_theta) 
            map_y     =  best_y - ox*sin(o_theta) - oy*cos(o_theta)
            map_theta =  best_theta - o_theta

            [map_x]   = R(-o_theta) * [ox, oy] restado de [best_x, best_y]
        """
        # Pose actual que reporta la odometría (odom → base_link)
        ox, oy, o_theta = self.current_odom_pose

        # Rotación inversa: llevar el vector odom al marco map
        cos_t = math.cos(self.best_theta - o_theta)
        sin_t = math.sin(self.best_theta - o_theta)

        # T_map_odom = T_map_base ⊖ T_odom_base
        map_odom_x     =  self.best_x - (ox * math.cos(self.best_theta - o_theta)
                                        - oy * math.sin(self.best_theta - o_theta))
        map_odom_y     =  self.best_y - (ox * math.sin(self.best_theta - o_theta)
                                        + oy * math.cos(self.best_theta - o_theta))
        map_odom_theta =  self.best_theta - o_theta

        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = "pumas_map"
        t.child_frame_id  = "pumas_odom"

        t.transform.translation.x = map_odom_x
        t.transform.translation.y = map_odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(math.sin(map_odom_theta / 2.0))
        t.transform.rotation.w = float(math.cos(map_odom_theta / 2.0))

        self.tf_broadcaster.sendTransform(t)
        
    def read_yaml(self,config_file):
        with open(config_file, 'r') as file:
            configs = yaml.safe_load(file)
            self. map_name = configs["name"]
            self.FIELD_X_MIN = configs["field"]["x_min"] 
            self.FIELD_X_MAX = configs["field"]["x_max"] 
            self.FIELD_Y_MIN = configs["field"]["y_min"] 
            self.FIELD_Y_MAX = configs["field"]["y_max"]
            
            for _ , landmark in configs["landmarks"].items():
                self.map_landmarks[landmark["id"]]=[]
                for point in landmark["points"]:
                    self.map_landmarks[landmark["id"]].append(tuple(point))
            #print(self.map_landmarks)

    def init_particles(self):
        self.particles = []
        self.num_particles = 800
    
    # Start pose
        init_x     = self.best_x
        init_y     = self.best_y
        init_theta = self.best_theta   # rad orientation
    
    # Start point
        sigma_xy    = 0.5
        sigma_theta = math.radians(20)
    
        for _ in range(self.num_particles):
            x = random.gauss(init_x, sigma_xy)
            y = random.gauss(init_y, sigma_xy)
            theta = random.gauss(init_theta, sigma_theta)
        
            # Clamping
            x = max(self.FIELD_X_MIN, min(x, self.FIELD_X_MAX))
            y = max(self.FIELD_Y_MIN, min(y, self.FIELD_Y_MAX))
        
            self.particles.append([x, y, theta])
    
        self.weights       = [1.0 / self.num_particles] * self.num_particles
        self.particle_scores = [1.0 / self.num_particles] * self.num_particles

# -----------MOTION MODEL -------------------------
# Based on Table 5.6 page 136 PR

    def tf_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'pumas_odom',
                'pumas_base_link',
                rclpy.time.Time())
            self.curr_pose= [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    math.atan2(t.transform.rotation.z, t.transform.rotation.w)*2
                    ]
        except (LookupException):
            self.get_logger().info('Waiting for pumas_map->pumas_base_link')
            return
        except TransformException as ex:
            self.get_logger().error(f'TF2 exception: {ex}')
            return 
        curr_pose = self.curr_pose
        self.current_odom_pose = curr_pose
        
        curr_time = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
	
        if self.last_odom_pose is None:
            self.last_odom_pose = curr_pose
            self.last_odom_time = curr_time
            return

        # 1. Calculate Deltas (Algorithm Table 5.6 lines 2-4)
        u_t = self.calculate_deltas(curr_pose, self.last_odom_pose)
        delta_rot1, delta_trans_odom, delta_rot2 = u_t
        
        dt = curr_time - self.last_odom_time
        dt = max(dt, 1e-4)
        
        if delta_trans_odom >0.01:
            delta_trans = delta_trans_odom
            # delta_trans = self.forward_speed * dt
            self.is_moving = True
        elif abs(delta_rot1 + delta_rot2)>0.02:
            delta_trans = 0.0
            self.is_moving = True
        else:
            self.is_moving = False
            
        # 2. Update particles only if there is significant motion
        if self.is_moving:
            u_t_corrected = (delta_rot1, delta_trans, delta_rot2)
            new_particles = []
            for p in self.particles:
                # Algorithm 5.4 lines 5-11
                new_p = self.sample_motion_model(u_t_corrected, p, self.alphas)
                new_particles.append(new_p)
            self.particles = new_particles
            self.publish_particles()
        
        self.last_odom_pose = curr_pose
        self.last_odom_time = curr_time
        
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
        x_new = max(self.FIELD_X_MIN - 0.1, min(x_new, self.FIELD_X_MAX + 0.1))
        y_new = max(self.FIELD_Y_MIN - 0.1, min(y_new, self.FIELD_Y_MAX + 0.1))
        return [x_new, y_new, angle_diff(theta_new, 0)]

#------------OBSERVATION MODEL---------------------
    def observation_callback(self, msg):
        if not self.is_moving:
            return
        self.latest_observations = sorted(list(msg.landmarks), key=lambda l: l.angle)

        valid_landmarks = [lm for lm in msg.landmarks if lm.id not in [0, 2]]
        self.latest_observations = sorted(list(msg.landmarks), key=lambda l: l.angle)

        # Si hay menos de 3 landmarks, solo confiar en odometría
        if len(self.latest_observations) < 3:
            #self.get_logger().info(f"Only {len(self.latest_observations)} landmarks — trusting odometry only.")
            self.publish_particles()
            self.publish_estimated_pose()
            return

        if self.latest_observations:
            #  CALCULAR PESOS (Update Step)
            new_weights = []
            for idx, p in enumerate(self.particles):
                preds = self.predict_measurements(p)
                instant_weight = self.similarity_function(preds, self.latest_observations)
                
                # NUEVO: Filtro de memoria (Inercia de peso)
                # 70% peso histórico, 30% lo que ve la cámara ahorita
                if idx < len(self.particle_scores):
                    smoothed_weight = 0.7 * self.particle_scores[idx] + 0.3 * instant_weight
                else:
                    smoothed_weight = instant_weight
                
                new_weights.append(smoothed_weight)

            # Guardamos los suavizados en la memoria para el próximo ciclo
            self.particle_scores = new_weights.copy()

            # Check average of weights
            avg_weight = sum(new_weights)/len(new_weights)
            max_weight = max(new_weights)

            # DETECCIÓN DE DIVERGENCIA SEVERA
            if max_weight < 1e-5:  # Pesos muy bajos
                self.get_logger().warn(
                    f"Divergence detected - Max weight: {max_weight:.6f}"
                )
                # Inyectar % de partículas aleatorias
                num_random = int(0.1 * self.num_particles)
                for i in range(num_random):
                    self.particles[i] = [
                        random.uniform(-self.field_x/2, self.field_x/2),
                        random.uniform(-self.field_y/2, self.field_y/2),
                        random.uniform(-math.pi, math.pi)
                    ]
                self.get_logger().info(f"Looking for position...")
            #  NORMALIZAR PESOS (Total Probability Theorem) 
            sum_w = sum(new_weights) + 1e-8
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

            # self.weights (recién normalizados, ANTES del resample-reset) ya
            # representan "lo que ve el robot" -> se usan dentro de
            # compute_best_pose() para la fusión Kalman contra el promedio
            # de movimiento puro.
            self.compute_best_pose()
            kx, ky, kth = self.kf_gain
            self.get_logger().info(
                f"KF gain -> x:{kx:.2f} y:{ky:.2f} theta:{kth:.2f} "
                f"(cercano a 1 = confía en visión, cercano a 0 = confía en movimiento)"
            )
            self.publish_particles()
            self.publish_estimated_pose()
        else:
            return

    def publish_particles(self):
        msg = PoseArray()
        msg.header.frame_id = "pumas_map"
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
    
    def resample(self, robot_is_moving):
        M = len(self.particles)
        
        # 1. ELITISMO: Encontrar los índices de las mejores partículas antes de destruir los pesos
        best_indices = sorted(range(M), key=lambda k: self.weights[k], reverse=True)
        num_elites = int(M * 0.03)  # Conservar el 3% de las mejores partículas
        elites = [self.particles[idx] for idx in best_indices[:num_elites]]
        
        # 2. ALGORITMO LOW VARIANCE SAMPLER (Para llenar el resto de la población)
        new_particles = []
        r = random.uniform(0, 1.0 / M)
        c = self.weights[0]
        i = 0
        
        for m in range(1, (M - num_elites) + 1):
            U = r + (m - 1) * (1.0 / M)
            while U > c:
                i = (i + 1) % M
                c += self.weights[i]
                
            p = self.particles[i]
            
            if not robot_is_moving:
                jitter_xy = 0.001
                jitter_theta = 0.001
            else:
                jitter_xy = 0.01
                jitter_theta = 0.01
            
            nx = p[0] + random.gauss(0, jitter_xy)
            ny = p[1] + random.gauss(0, jitter_xy)
            nt = angle_diff(p[2], random.gauss(0, jitter_theta))
            
            # CLAMPING
            nx = max(self.FIELD_X_MIN, min(nx, self.FIELD_X_MAX))
            ny = max(self.FIELD_Y_MIN, min(ny, self.FIELD_Y_MAX))

            new_particles.append([nx, ny, nt])
            
        # 3. INYECTAR LAS ELITES
        self.particles = elites + new_particles
        
        # Reiniciamos la memoria de pesos para la nueva generación
        self.weights = [1.0 / M] * M
        self.particle_scores = [1.0 / M] * M

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
            return 1e-6

        num_matches = len(matched_errors)
        num_observations = len(observations)
        num_predictions = len(predicted_dets)
        #------ Gaussian Similarity--------
        similarity = 0.0
        var = self.sigma_angle ** 2
        for err in matched_errors:
            lklihood = -(err**2)/(2*var)
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
