import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# ============================================================================
# CLASE 1: SOLVER DE LA MATRIZ DE MASA (DINÁMICA INVERSA CORREGIDA)
# ============================================================================
class RoverMassMatrixSolver:
    def __init__(self):
        # 1. PARÁMETROS FÍSICOS 
        self.m2, self.m3, self.m4, self.m5 = 0.975, 1.384, 0.581, 0.406
        self.L2, self.L3, self.L4 = 0.45, 0.43, 0.24 
        self.Lc2, self.Lc3, self.Lc4, self.Lc5 = 0.25, 0.23, 0.18, 0.10
        self.I1, self.I2, self.I3, self.I4, self.I5 = 0.01, 0.01, 0.01, 0.01, 0.01

        self.C22 = 1 + 0*(self.I2 + (self.L2**2)*(self.m3 + self.m4 + self.m5) + (self.Lc2**2)*self.m2)
        self.C33 = 1 + 0*(self.I3 + (self.L3**2)*(self.m4 + self.m5) + (self.Lc3**2)*self.m3)
        self.C44 = 20*(self.m5*(self.L4**2) + self.m4*(self.Lc4**2) + self.I4)
        self.C55 = 70*(self.m5*(self.Lc5**2) + self.I5)

        self.C_s7 = self.L4*self.m5 + self.Lc4*self.m4
        self.C_s1 = self.L2 * (self.L3*self.m4 + self.L3*self.m5 + self.Lc3*self.m3)
        self.C_s2 = self.L4*self.Lc5*self.m5
        self.C_s3 = self.L3*self.Lc5*self.m5
        self.C_s4 = self.L2*self.Lc5*self.m5
        self.C_s5 = self.L3 * self.C_s7
        self.C_s6 = self.L2 * self.C_s7

        self.k_q2 = (self.L2**2)*(self.m3 + self.m4 + self.m5) + (self.Lc2**2)*self.m2
        self.k_q3 = (self.L3**2)*(self.m4 + self.m5) + (self.Lc3**2)*self.m3
        self.k_q4 = (self.L4**2)*self.m5 + (self.Lc4**2)*self.m4
        self.k_q5 = (self.Lc5**2)*self.m5
        self.k_23 = 2 * self.L2 * (self.L3*(self.m4 + self.m5) + self.Lc3*self.m3)
        self.k_24 = 2 * self.L2 * (self.L4*self.m5 + self.Lc4*self.m4)
        self.k_34 = 2 * self.L3 * (self.L4*self.m5 + self.Lc4*self.m4)
        self.k_25 = 2 * self.L2 * self.Lc5 * self.m5
        self.k_35 = 2 * self.L3 * self.Lc5 * self.m5
        self.k_45 = 2 * self.L4 * self.Lc5 * self.m5

    def get_decoupling_torque(self, q, v_virtual, u0):
        q2, q3, q4, q5 = q[1], q[2], q[3], q[4]
        s2, s3, s4, s5 = math.sin(q2), math.sin(q3), math.sin(q4), math.sin(q5)
        
        M11 = 100*(self.I1 + \
              self.k_q2*(s2**2) + self.k_q3*(s3**2) + self.k_q4*(s4**2) + self.k_q5*(s5**2) + \
              self.k_23*(s2*s3) + self.k_24*(s2*s4) + self.k_34*(s3*s4) + \
              self.k_25*(s2*s5) + self.k_35*(s3*s5) + self.k_45*(s4*s5))

        s_1 = self.C_s1 * math.cos(q2 - q3)*1
        s_2 = self.C_s2 * math.cos(q4 - q5)*1
        s_3 = self.C_s3 * math.cos(q3 - q5)*1
        s_4 = self.C_s4 * math.cos(q2 - q5)*1
        s_5 = self.C_s5 * math.cos(q3 - q4)*1
        s_6 = self.C_s6 * math.cos(q2 - q4)*1

        M_4x4 = np.array([
            [self.C22, s_1,      s_6,      s_4     ],
            [s_1,      self.C33, s_5,      s_3     ],
            [s_6,      s_5,      self.C44, s_2     ],
            [s_4,      s_3,      s_2,      self.C55]
        ], dtype=np.float64)

        u_real = np.zeros(5)
        u_real[0] = u0[0] + (v_virtual[0] * M11)
        u_real[1:5] = u0[1:5] + np.dot(M_4x4, v_virtual[1:5])

        return u_real

# ============================================================================
# CLASE 2: NODO PRINCIPAL ADRC CON GENERADOR DE TRAYECTORIA QUÍNTICA
# ============================================================================
class RoverArmFullADRC(Node):
    def __init__(self):
        super().__init__('rover_arm_full_adrc')
        self.kinematics = RoverMassMatrixSolver()
        self.u0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        # 1. Suscriptor de estados actuales
        self.subscription = self.create_subscription(
            JointState,
            '/world/empty/model/mi_rover/joint_state',
            self.sensor_callback,
            10)
            
        # 2. Suscriptor para recibir los comandos objetivo de cada joint
        self.sub_targets = self.create_subscription(
            JointState,
            '/target_joint_states',
            self.target_callback,
            10)

        # 3. Publicadores
        self.pubs_brazo = {}
        for i in range(1, 6):
            joint_name = f'Joint_{i}'
            topic_name = f'/model/mi_rover/joint/{joint_name}/cmd_force'
            self.pubs_brazo[joint_name] = self.create_publisher(Float64, topic_name, 10)
        self.pub_rover = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_dt = 0.02  # 50 Hz
        
        self.motor_specs = {
            'Joint_1': {'safe_tq': 2.50,  'Kt': 0.284}, 
            'Joint_2': {'safe_tq': 21.00, 'Kt': 3.024}, 
            'Joint_3': {'safe_tq': 21.00, 'Kt': 3.024}, 
            'Joint_4': {'safe_tq': 2.50,  'Kt': 0.250}, 
            'Joint_5': {'safe_tq': 2.00,  'Kt': 0.660}  
        }

        self.adrc_params = {
            'Joint_1': {'wn': 1.0, 'eps': 0.2, 'zi': 1.0, 'B': 1.0}, 
            'Joint_2': {'wn': 7.0, 'eps': 0.2, 'zi': 1.0, 'B': 1.0}, 
            'Joint_3': {'wn': 3.0, 'eps': 0.2, 'zi': 1.0, 'B': 1.0},
            'Joint_4': {'wn': 2.0, 'eps': 0.2, 'zi': 1.0, 'B': 1.0},
            'Joint_5': {'wn': 1.5, 'eps': 0.2, 'zi': 1.0, 'B': 1.0}
        }
        
        self.k0, self.k1, self.k2, self.k3 = {}, {}, {}, {}
        self.s1_adrc = {f'Joint_{i}': 0.0 for i in range(1, 6)}
        self.s2_adrc = {f'Joint_{i}': 0.0 for i in range(1, 6)}

        for joint, params in self.adrc_params.items():
            wn, eps, zi = params['wn'], params['eps'], params['zi']
            wnp = wn / eps
            G1, G0 = 2.0 * zi * wn, wn**2
            L1, L0 = 2.0 * zi * wnp, wnp**2
            self.k0[joint] = G0 * L0
            self.k1[joint] = L0 * G1 + G0 * L1
            self.k2[joint] = L0 + G0 + L1 * G1
            self.k3[joint] = L1 + G1

        # Variables de estado y trayectoria
        self.current_pos = {f'Joint_{i}': 0.0 for i in range(1, 6)}
        self.target_pos = {f'Joint_{i}': 0.0 for i in range(1, 6)}
        
        # Parámetros para la generación de la trayectoria suave (Polinomio de 5to grado)
        self.trajectory_active = False
        self.t_traj_start = 0.0
        self.t_traj_duration = 3.0 # <-- Tiempo FIJO en 3 segundos
        self.start_pos = {f'Joint_{i}': 0.0 for i in range(1, 6)}
        self.end_pos = {f'Joint_{i}': 0.0 for i in range(1, 6)}

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(self.target_dt, self.control_loop)
        self.get_logger().info("ADRC MIMO con Desacoplamiento y Trayectoria Quíntica inicializado.")

    @staticmethod
    def quintic_trajectory(t, t_start, t_end, q_start, q_end):
        if t <= t_start:
            return q_start
        elif t >= t_end:
            return q_end
        
        tau = (t - t_start) / (t_end - t_start)
        s = 10 * (tau ** 3) - 15 * (tau ** 4) + 6 * (tau ** 5)
        q_t = q_start + (q_end - q_start) * s
        return q_t

    def sensor_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_pos:
                self.current_pos[name] = msg.position[i]

    def target_callback(self, msg):
        self.t_traj_start = self.get_clock().now().nanoseconds * 1e-9
        for i, name in enumerate(msg.name):
            if name in self.end_pos:
                self.start_pos[name] = self.current_pos[name]
                self.end_pos[name] = msg.position[i]
        
        self.trajectory_active = True
        self.get_logger().info(f"Nuevo target recibido. Calculando trayectoria suave A->B en {self.t_traj_duration}s...")

    def control_loop(self):
        current_time_obj = self.get_clock().now()
        current_time_sec = current_time_obj.nanoseconds * 1e-9
        
        dt_duration = current_time_obj - self.last_time
        dt = dt_duration.nanoseconds * 1e-9
        self.last_time = current_time_obj

        if dt <= 1e-6:
            return

        if self.trajectory_active:
            u_time = current_time_sec - self.t_traj_start
            
            for i in range(1, 6):
                j_name = f'Joint_{i}'
                y_traj = self.quintic_trajectory(
                    t=u_time,
                    t_start=0.0,
                    t_end=self.t_traj_duration,
                    q_start=self.start_pos[j_name],
                    q_end=self.end_pos[j_name]
                )
                self.target_pos[j_name] = y_traj
            
            if u_time > self.t_traj_duration:
                self.trajectory_active = False
                self.get_logger().info("Trayectoria completada en 3s. Estabilizando en el Punto B.")

        v_virtual = np.zeros(5)
        q_actual = np.zeros(5)
        s2dot_array = np.zeros(5)

        for i in range(1, 6):
            j_name = f'Joint_{i}'
            q_actual[i-1] = self.current_pos[j_name]
            
            e_adrc = self.current_pos[j_name] - self.target_pos[j_name]
            s2dot = -self.k3[j_name] * self.s2_adrc[j_name] + e_adrc
            s2dot_array[i-1] = s2dot 
            
            B_val = self.adrc_params[j_name]['B']
            v = -(self.k2[j_name] * s2dot + self.k1[j_name] * self.s2_adrc[j_name] + self.k0[j_name] * self.s1_adrc[j_name]) / B_val
            v_virtual[i-1] = v

        u_real = self.kinematics.get_decoupling_torque(q_actual, v_virtual, self.u0)

        for i in range(1, 6):
            j_name = f'Joint_{i}'
            safe_limit = self.motor_specs[j_name]['safe_tq']
            
            u_real[i-1] = np.clip(u_real[i-1], -safe_limit, safe_limit)
            
            self.s1_adrc[j_name] += self.s2_adrc[j_name] * dt
            self.s2_adrc[j_name] += s2dot_array[i-1] * dt
            
            self.s1_adrc[j_name] = np.clip(self.s1_adrc[j_name], -safe_limit, safe_limit)
            self.s2_adrc[j_name] = np.clip(self.s2_adrc[j_name], -safe_limit, safe_limit)

            msg_torque = Float64()
            msg_torque.data = float(u_real[i-1]) 
            self.pubs_brazo[j_name].publish(msg_torque)

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_rover.publish(cmd)

        if not hasattr(self, 'print_counter'):
            self.print_counter = 0
            
        self.print_counter += 1
        
        # --- MODIFICACIÓN DE LA TELEMETRÍA ---
        if self.print_counter >= 2: 
            x_deg = [np.rad2deg(self.current_pos[f'Joint_{i}']) for i in range(1, 6)]
            tar_deg = [np.rad2deg(self.target_pos[f'Joint_{i}']) for i in range(1, 6)]
            
            log_msg = (
                f"\n--- ESTADOS (Grados) | OBJETIVO (Grados) | ESFUERZO INYECTADO (Nm) ---\n"
                f"J1 -> Pos: {x_deg[0]:>8.2f}° | Target: {tar_deg[0]:>8.2f}° | Torque: {u_real[0]:>8.2f}\n"
                f"J2 -> Pos: {x_deg[1]:>8.2f}° | Target: {tar_deg[1]:>8.2f}° | Torque: {u_real[1]:>8.2f}\n"
                f"J3 -> Pos: {x_deg[2]:>8.2f}° | Target: {tar_deg[2]:>8.2f}° | Torque: {u_real[2]:>8.2f}\n"
                f"J4 -> Pos: {x_deg[3]:>8.2f}° | Target: {tar_deg[3]:>8.2f}° | Torque: {u_real[3]:>8.2f}\n"
                f"J5 -> Pos: {x_deg[4]:>8.2f}° | Target: {tar_deg[4]:>8.2f}° | Torque: {u_real[4]:>8.2f}\n"
                f"-----------------------------------------------------------------------"
            )
            self.get_logger().info(log_msg)
            self.print_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = RoverArmFullADRC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
