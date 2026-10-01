#include <chrono>
#include <cmath>
#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// ============================================================================
// CLASE 1: SOLVER DE LA MATRIZ DE MASA (DINÁMICA INVERSA CORREGIDA)
// ============================================================================
class RoverMassMatrixSolver {
private:
    double m2 = 0.975, m3 = 1.384, m4 = 0.581, m5 = 0.406;
    double L2 = 0.45, L3 = 0.43, L4 = 0.24;
    double Lc2 = 0.25, Lc3 = 0.23, Lc4 = 0.18, Lc5 = 0.10;
    double I1 = 0.01, I2 = 0.01, I3 = 0.01, I4 = 0.01, I5 = 0.01;

    double C22, C33, C44, C55;
    double C_s7, C_s1, C_s2, C_s3, C_s4, C_s5, C_s6;
    double k_q2, k_q3, k_q4, k_q5;
    double k_23, k_24, k_34, k_25, k_35, k_45;

public:
    RoverMassMatrixSolver() {
        C22 = 2.0 * (I2 + (L2 * L2) * (m3 + m4 + m5) + (Lc2 * Lc2) * m2);
        C33 = 2.0 * (I3 + (L3 * L3) * (m4 + m5) + (Lc3 * Lc3) * m3);
        C44 = 20.0 * (m5 * (L4 * L4) + m4 * (Lc4 * Lc4) + I4);
        C55 = 70.0 * (m5 * (Lc5 * Lc5) + I5);

        C_s7 = L4 * m5 + Lc4 * m4;
        C_s1 = L2 * (L3 * m4 + L3 * m5 + Lc3 * m3);
        C_s2 = L4 * Lc5 * m5;
        C_s3 = L3 * Lc5 * m5;
        C_s4 = L2 * Lc5 * m5;
        C_s5 = L3 * C_s7;
        C_s6 = L2 * C_s7;

        k_q2 = (L2 * L2) * (m3 + m4 + m5) + (Lc2 * Lc2) * m2;
        k_q3 = (L3 * L3) * (m4 + m5) + (Lc3 * Lc3) * m3;
        k_q4 = (L4 * L4) * m5 + (Lc4 * Lc4) * m4;
        k_q5 = (Lc5 * Lc5) * m5;
        
        k_23 = 2.0 * L2 * (L3 * (m4 + m5) + Lc3 * m3);
        k_24 = 2.0 * L2 * (L4 * m5 + Lc4 * m4);
        k_34 = 2.0 * L3 * (L4 * m5 + Lc4 * m4);
        k_25 = 2.0 * L2 * Lc5 * m5;
        k_35 = 2.0 * L3 * Lc5 * m5;
        k_45 = 2.0 * L4 * Lc5 * m5;
    }

    std::array<double, 5> get_decoupling_torque(const std::array<double, 5>& q, const std::array<double, 5>& v_virtual, const std::array<double, 5>& u0) {
        double q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4];
        double s2 = std::sin(q2), s3 = std::sin(q3), s4 = std::sin(q4), s5 = std::sin(q5);

        double M11 = 200.0 * (I1 +
            k_q2 * (s2 * s2) + k_q3 * (s3 * s3) + k_q4 * (s4 * s4) + k_q5 * (s5 * s5) +
            k_23 * (s2 * s3) + k_24 * (s2 * s4) + k_34 * (s3 * s4) +
            k_25 * (s2 * s5) + k_35 * (s3 * s5) + k_45 * (s4 * s5));

        double s_1 = C_s1 * std::cos(q2 - q3);
        double s_2 = C_s2 * std::cos(q4 - q5);
        double s_3 = C_s3 * std::cos(q3 - q5);
        double s_4 = C_s4 * std::cos(q2 - q5);
        double s_5 = C_s5 * std::cos(q3 - q4);
        double s_6 = C_s6 * std::cos(q2 - q4);

        std::array<double, 5> u_real;
        u_real[0] = u0[0] + (v_virtual[0] * M11);
        
        u_real[1] = u0[1] + (C22 * v_virtual[1] + s_1 * v_virtual[2] + s_6 * v_virtual[3] + s_4 * v_virtual[4]);
        u_real[2] = u0[2] + (s_1 * v_virtual[1] + C33 * v_virtual[2] + s_5 * v_virtual[3] + s_3 * v_virtual[4]);
        u_real[3] = u0[3] + (s_6 * v_virtual[1] + s_5 * v_virtual[2] + C44 * v_virtual[3] + s_2 * v_virtual[4]);
        u_real[4] = u0[4] + (s_4 * v_virtual[1] + s_3 * v_virtual[2] + s_2 * v_virtual[3] + C55 * v_virtual[4]);

        return u_real;
    }
};

// ============================================================================
// CLASE 2: NODO PRINCIPAL ADRC CON GENERADOR DE TRAYECTORIA QUÍNTICA
// ============================================================================
class RoverArmFullADRC : public rclcpp::Node {
private:
    RoverMassMatrixSolver kinematics_;
    std::array<double, 5> u0_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_states_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_targets_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 5> pubs_brazo_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_rover_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<double, 5> safe_tq_ = {2.50, 30.00, 21.00, 2.50, 2.00};
    std::array<double, 5> wn_      = {1.0, 4.3, 5.0, 1.5, 1.0};
    double eps_ = 0.1, zi_ = 1.0, B_ = 1.0;

    std::array<double, 5> k0_, k1_, k2_, k3_;
    std::array<double, 5> s1_adrc_ = {0.0}, s2_adrc_ = {0.0};

    std::array<double, 5> current_pos_ = {0.0};
    std::array<double, 5> target_pos_  = {0.0};
    std::array<double, 5> start_pos_   = {0.0};
    std::array<double, 5> end_pos_     = {0.0};

    bool trajectory_active_ = false;
    double t_traj_start_ = 0.0;
    const double t_traj_duration_ = 3.0;

    rclcpp::Time last_time_;
    int print_counter_ = 0;

    int get_joint_index(const std::string& name) {
        if (name == "Joint_1") return 0;
        if (name == "Joint_2") return 1;
        if (name == "Joint_3") return 2;
        if (name == "Joint_4") return 3;
        if (name == "Joint_5") return 4;
        return -1;
    }

    double quintic_trajectory(double t, double t_start, double t_end, double q_start, double q_end) {
        if (t <= t_start) return q_start;
        if (t >= t_end) return q_end;
        
        double tau = (t - t_start) / (t_end - t_start);
        double s = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
        return q_start + (q_end - q_start) * s;
    }

    void sensor_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            int idx = get_joint_index(msg->name[i]);
            if (idx != -1) current_pos_[idx] = msg->position[i];
        }
    }

    void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        t_traj_start_ = this->now().seconds();
        for (size_t i = 0; i < msg->name.size(); ++i) {
            int idx = get_joint_index(msg->name[i]);
            if (idx != -1) {
                start_pos_[idx] = current_pos_[idx];
                end_pos_[idx] = msg->position[i];
            }
        }
        trajectory_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Nuevo target recibido. Calculando trayectoria suave A->B en 3.0s...");
    }

    void control_loop() {
        auto current_time_obj = this->now();
        double dt = (current_time_obj - last_time_).seconds();
        last_time_ = current_time_obj;

        if (dt <= 1e-6) return;

        double current_time_sec = current_time_obj.seconds();

        if (trajectory_active_) {
            double u_time = current_time_sec - t_traj_start_;
            for (int i = 0; i < 5; ++i) {
                target_pos_[i] = quintic_trajectory(u_time, 0.0, t_traj_duration_, start_pos_[i], end_pos_[i]);
            }
            if (u_time > t_traj_duration_) {
                trajectory_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Trayectoria completada en 3s. Estabilizando en el Punto B.");
            }
        }

        std::array<double, 5> v_virtual = {0.0};
        std::array<double, 5> s2dot_array = {0.0};

        for (int i = 0; i < 5; ++i) {
            double e_adrc = current_pos_[i] - target_pos_[i];
            double s2dot = -k3_[i] * s2_adrc_[i] + e_adrc;
            s2dot_array[i] = s2dot;
            
            v_virtual[i] = -(k2_[i] * s2dot + k1_[i] * s2_adrc_[i] + k0_[i] * s1_adrc_[i]) / B_;
        }

        std::array<double, 5> u_real = kinematics_.get_decoupling_torque(current_pos_, v_virtual, u0_);

        // Usamos un dt_control fijo de 0.01s (100 Hz) para la integración numérica
        // Esto evita que el jitter del sistema operativo desestabilice el ADRC
        const double dt_control = 0.01;

        for (int i = 0; i < 5; ++i) {
            double safe_limit = safe_tq_[i];
            u_real[i] = std::clamp(u_real[i], -safe_limit, safe_limit);
            
            // Integración de Euler estabilizada
            s1_adrc_[i] += s2_adrc_[i] * dt_control;
            s2_adrc_[i] += s2dot_array[i] * dt_control;            
            s1_adrc_[i] = std::clamp(s1_adrc_[i], -safe_limit, safe_limit);
            s2_adrc_[i] = std::clamp(s2_adrc_[i], -safe_limit, safe_limit);

            std_msgs::msg::Float64 msg_torque;
            msg_torque.data = u_real[i];
            pubs_brazo_[i]->publish(msg_torque);
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pub_rover_->publish(cmd);

        print_counter_++;
        if (print_counter_ >= 5) {
            RCLCPP_INFO(this->get_logger(), 
                "\n--- ESTADOS (Grados) | OBJETIVO (Grados) | ESFUERZO INYECTADO (Nm) ---\n"
                "J1 -> Pos: %8.2f° | Target: %8.2f° | Torque: %8.2f\n"
                "J2 -> Pos: %8.2f° | Target: %8.2f° | Torque: %8.2f\n"
                "J3 -> Pos: %8.2f° | Target: %8.2f° | Torque: %8.2f\n"
                "J4 -> Pos: %8.2f° | Target: %8.2f° | Torque: %8.2f\n"
                "J5 -> Pos: %8.2f° | Target: %8.2f° | Torque: %8.2f\n"
                "-----------------------------------------------------------------------",
                current_pos_[0] * 180.0 / M_PI, target_pos_[0] * 180.0 / M_PI, u_real[0],
                current_pos_[1] * 180.0 / M_PI, target_pos_[1] * 180.0 / M_PI, u_real[1],
                current_pos_[2] * 180.0 / M_PI, target_pos_[2] * 180.0 / M_PI, u_real[2],
                current_pos_[3] * 180.0 / M_PI, target_pos_[3] * 180.0 / M_PI, u_real[3],
                current_pos_[4] * 180.0 / M_PI, target_pos_[4] * 180.0 / M_PI, u_real[4]
            );
            print_counter_ = 0;
        }
    }

public:
    RoverArmFullADRC() : Node("rover_arm_full_adrc") {
        sub_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/world/empty/model/mi_rover/joint_state", 10,
            std::bind(&RoverArmFullADRC::sensor_callback, this, std::placeholders::_1));
            
        sub_targets_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/target_joint_states", 10,
            std::bind(&RoverArmFullADRC::target_callback, this, std::placeholders::_1));

        for (int i = 0; i < 5; ++i) {
            std::string topic = "/model/mi_rover/joint/Joint_" + std::to_string(i+1) + "/cmd_force";
            pubs_brazo_[i] = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
        }
        
        pub_rover_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        for (int i = 0; i < 5; ++i) {
            double wnp = wn_[i] / eps_;
            double G1 = 2.0 * zi_ * wn_[i];
            double G0 = wn_[i] * wn_[i];
            double L1 = 2.0 * zi_ * wnp;
            double L0 = wnp * wnp;
            
            k0_[i] = G0 * L0;
            k1_[i] = L0 * G1 + G0 * L1;
            k2_[i] = L0 + G0 + L1 * G1;
            k3_[i] = L1 + G1;
        }

        last_time_ = this->now();
        timer_ = this->create_wall_timer(10ms, std::bind(&RoverArmFullADRC::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "ADRC MIMO con Desacoplamiento y Trayectoria Quíntica inicializado en C++ a 100 Hz.");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverArmFullADRC>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
