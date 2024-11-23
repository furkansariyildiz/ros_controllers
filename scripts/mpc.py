#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import casadi as ca
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

        # MPC parametreleri
        self.dt = 0.1       # Zaman adımı
        self.N = 20         # Öngörü ufku
        self.frequency = 10  # Kontrol döngüsü frekansı (Hz)

        # Durum ve kontrol değişkenleri
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        self.states = ca.vertcat(x, y, theta)
        n_states = self.states.size1()

        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        self.controls = ca.vertcat(v, omega)
        n_controls = self.controls.size1()

        # Sistem dinamiği (unicycle modeli)
        rhs = ca.vertcat(v * ca.cos(theta),
                         v * ca.sin(theta),
                         omega)

        # Dinamiği fonksiyon haline getirelim
        self.f = ca.Function('f', [self.states, self.controls], [rhs])

        # MPC değişkenleri
        self.U = ca.SX.sym('U', n_controls, self.N)          # Kontrol değişkenleri
        self.X = ca.SX.sym('X', n_states, self.N+1)          # Durum değişkenleri
        self.P = ca.SX.sym('P', n_states + self.N*n_states)  # Parametre vektörü

        # Amaç fonksiyonu ve kısıtlar
        obj = 0     # Amaç fonksiyonu
        g = []      # Kısıtlar listesi

        # Başlangıç durumu kısıtı
        st = self.X[:, 0]
        g.append(st - self.P[0:n_states])  # x0 = x_initial

        # Maliyet fonksiyonu (durum ve kontrol ağırlıkları)
        Q = ca.diagcat(20, 20, 1.0)        # Durum ağırlıkları
        R = ca.diagcat(0.5, 0.05)          # Kontrol ağırlıkları

        # MPC döngüsü
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            st_next = self.X[:, k+1]
            f_value = self.f(st, con)
            st_next_euler = st + self.dt * f_value
            g.append(st_next - st_next_euler)  # Sistem dinamiği kısıtı

            # Referans durumu
            ref = self.P[n_states + k*n_states : n_states + (k+1)*n_states]

            # Amaç fonksiyonu güncelleme
            obj = obj + ca.mtimes([(st - ref).T, Q, (st - ref)]) + ca.mtimes([con.T, R, con])

        # Değişkenleri tek bir vektörde birleştirelim
        OPT_variables = ca.vertcat(ca.reshape(self.U, -1, 1), ca.reshape(self.X, -1, 1))

        # Kısıtları birleştirelim
        g = ca.vertcat(*g)

        # NLP problemi tanımlayalım
        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': g, 'p': self.P}

        # Çözümleyici ayarları
        opts = {'ipopt.max_iter': 2000,
                'ipopt.print_level': 0,
                'print_time': 0,
                'ipopt.acceptable_tol': 1e-8,
                'ipopt.acceptable_obj_change_tol': 1e-6}

        # Çözümleyiciyi oluştur
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        # Kontrol ve durum tahminleri için başlangıç değerleri
        self.u0 = np.zeros((self.N, n_controls))            # Kontrol değişkenleri için başlangıç tahmini
        self.x_mpc = np.zeros((self.N+1, n_states))         # Durum değişkenleri için başlangıç tahmini

        # Durum ve kontrol sınırları
        self.lbx = np.full(OPT_variables.shape[0], -np.inf)
        self.ubx = np.full(OPT_variables.shape[0], np.inf)

        # Kontrol sınırları (TurtleBot3'ün fiziksel sınırlarına göre ayarlayın)
        v_max = 0.22  # m/s
        v_min = -v_max
        omega_max = 2.84  # rad/s
        omega_min = -omega_max

        for k in range(self.N):
            # Hız sınırları
            self.lbx[k*2] = v_min
            self.ubx[k*2] = v_max
            # Açısal hız sınırları
            self.lbx[k*2+1] = omega_min
            self.ubx[k*2+1] = omega_max

        # Mevcut durum
        self.x0 = np.zeros((n_states,))

        # Referans trajektori
        self.ref_traj = np.zeros((self.N, n_states))

        # ROS2 ayarları
        # Odometri aboneliği
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Kontrol komutu yayıncısı
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.position_ = Odometry()
        self.vehicle_positions_ = []
        self.reference_trajectory_ = []

        self.update_reference()


        # Kontrol döngüsünü başlat
        self.timer = self.create_timer(1.0 / self.frequency, self.control_loop)

    def odom_callback(self, msg):
        # Odometri verilerini al ve mevcut durumu güncelle
        position = msg.pose.pose.position
        self.position_ = msg
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)

        self.x0[0] = position.x
        self.x0[1] = position.y
        self.x0[2] = yaw

        self.vehicle_positions_.append((position.x, position.y, position.z))

    def control_loop(self):
        # Referans trajektoriyi güncelle
        self.update_reference()

        # Parametreleri güncelle
        args = {
            'p': np.concatenate([self.x0, self.ref_traj.reshape(-1)]),
            'x0': np.concatenate([self.u0.reshape(-1), self.x_mpc.reshape(-1)]),
            'lbg': np.zeros(self.X.shape[0]*(self.N+1)),
            'ubg': np.zeros(self.X.shape[0]*(self.N+1)),
            'lbx': self.lbx,
            'ubx': self.ubx
        }

        # Problemi çöz
        sol = self.solver(**args)
        sol_u = sol['x'][0:self.N*2].full().reshape(self.N, 2)
        sol_x = sol['x'][self.N*2:].full().reshape(self.N+1, 3)

        # İlk kontrol girdisini uygula
        u_applied = sol_u[0, :]
        cmd = Twist()
        cmd.linear.x = float(u_applied[0])
        cmd.angular.z = float(u_applied[1])
        self.cmd_pub.publish(cmd)

        # Kontrol ve durum tahminlerini güncelle
        self.u0 = np.vstack((sol_u[1:, :], sol_u[-1, :]))
        self.x_mpc = np.vstack((sol_x[1:, :], sol_x[-1, :]))

    def update_reference(self):
        # Sinüs dalga referans trajektorisi
        amplitude = 1.0     # Sinüs dalgasının genliği
        frequency = 0.5     # Sinüs dalgasının frekansı
        v_ref = 0.1         # x ekseninde referans hız

        if self.position_.pose.pose.position.x >= 5.0:
            print("Hedefe ulaşıldı!")
            self.save_data()
            return 

        for k in range(self.N):
            t = (k + 1) * self.dt  # Gelecekteki zaman adımı
            x_ref = self.x0[0] + v_ref * t
            y_ref = amplitude * np.sin(frequency * x_ref)
            
            # y'nin x'e göre türevi (dy/dx)
            dy_dx = amplitude * frequency * np.cos(frequency * x_ref)
            theta_ref = np.arctan2(dy_dx, 1)  # dx/dx = 1 olduğu varsayımıyla

            self.ref_traj[k, 0] = x_ref
            self.ref_traj[k, 1] = y_ref
            self.ref_traj[k, 2] = theta_ref

        for ref_state in self.ref_traj:
            self.reference_trajectory_.append((ref_state[0], ref_state[1], 0.0)) 



    def save_data(self):
        with open('/home/furkan/controller_ws/src/ros_controllers/results/result_mpc.csv', 'w') as f:
            f.write('vehicle_x,vehicle_y,vehicle_z\n')
            for pos in self.vehicle_positions_:
                f.write(f'{pos[0]},{pos[1]},{pos[2]}\n')

        with open('/home/furkan/controller_ws/src/ros_controllers/results/desired_mpc.csv', 'w') as f:
            f.write('desired_x,desired_y,desired_z\n')
            for ref in self.reference_trajectory_:
                f.write(f'{ref[0]},{ref[1]},{ref[2]}\n')


def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MPCController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
