#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include <cmath>    // Matematiksel fonksiyonlar için
#include <fstream>  // Sonuçları dosyaya kaydetmek için

int main() {
    using namespace casadi;

    // Zaman adımı ve öngörü ufku
    double dt = 0.1;  // Zaman adımı
    int N = 30;       // Öngörü ufku

    // Durum ve kontrol değişkenleri
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX theta = SX::sym("theta");
    SX states = SX::vertcat({x, y, theta});
    int n_states = states.size1();

    SX v = SX::sym("v");
    SX omega = SX::sym("omega");
    SX controls = SX::vertcat({v, omega});
    int n_controls = controls.size1();

    // Sistem dinamiği (unicycle modeli)
    SX rhs = SX::vertcat({
        v * SX::cos(theta),
        v * SX::sin(theta),
        omega
    });

    // Dinamiği fonksiyon haline getirelim
    Function f = Function("f", {states, controls}, {rhs});

    // MPC değişkenleri
    SX U = SX::sym("U", n_controls, N);          // Kontrol değişkenleri
    SX X = SX::sym("X", n_states, N + 1);        // Durum değişkenleri
    SX P = SX::sym("P", n_states + N * n_states);  // Parametre vektörü

    // Amaç fonksiyonu ve kısıtlar
    SX obj = 0;     // Amaç fonksiyonu
    std::vector<SX> g;      // Kısıtlar listesi

    // Başlangıç durumu kısıtı
    SX st = X(Slice(), 0);
    g.push_back(st - P(Slice(0, n_states)));  // x0 = x_initial

    // Maliyet fonksiyonu ağırlıkları
    SX Q = SX::diag(SX({20, 20, 0.05}));
    SX R = SX::diag(SX({0.5, 0.05}));

    // MPC döngüsü
    for (int k = 0; k < N; ++k) {
        st = X(Slice(), k);
        SX con = U(Slice(), k);
        SX st_next = X(Slice(), k + 1);
        SX f_value = f({st, con})[0];
        SX st_next_euler = st + dt * f_value;
        g.push_back(st_next - st_next_euler);  // Sistem dinamiği kısıtı

        // Referans durumu
        SX ref = P(Slice(n_states + k * n_states, n_states + (k + 1) * n_states));

        // Amaç fonksiyonu güncelleme
        obj = obj + SX::mtimes((st - ref).T(), SX::mtimes(Q, (st - ref))) + SX::mtimes(con.T(), SX::mtimes(R, con));
    }

    // Değişkenleri tek bir vektörde birleştirelim
    SX OPT_variables = SX::vertcat({
        SX::reshape(U, N * n_controls, 1),
        SX::reshape(X, (N + 1) * n_states, 1)
    });

    // Kısıtları birleştirelim
    SX g_concat = vertcat(g);

    // NLP problemi tanımlayalım
    Dict nlp_prob = {{"f", obj}, {"x", OPT_variables}, {"g", g_concat}, {"p", P}};

    // Çözümleyici ayarları
    Dict opts;
    opts["ipopt.max_iter"] = 2000;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.acceptable_tol"] = 1e-8;
    opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    // Çözümleyiciyi oluştur
    Function solver = nlpsol("solver", "ipopt", nlp_prob, opts);

    // Başlangıç durumu
    std::vector<double> x0 = {0.0, 0.0, 0.0};  // x, y, theta

    // Başlangıç tahminleri
    std::vector<double> u0(N * n_controls, 0.0);          // Kontrol değişkenleri için başlangıç tahmini
    std::vector<double> x_mpc((N + 1) * n_states, 0.0);   // Durum değişkenleri için başlangıç tahmini
    for (int i = 0; i < n_states; ++i) {
        x_mpc[i] = x0[i];
    }

    // MPC çözümleme döngüsü
    double sim_tim = 20.0;  // Simülasyon süresi (saniye)
    double t = 0.0;
    int mpc_iter = 0;
    std::vector<std::vector<double>> xx;        // Gerçek yol
    std::vector<std::vector<double>> u_cl;      // Kontrol girdileri
    std::vector<std::vector<double>> ref_traj_full;  // Tam referans yolu

    double frequency = 0.5;  // Sinüs dalgasının frekansı
    double amplitude = 1.0;  // Sinüs dalgasının genliği

    while (mpc_iter * dt < sim_tim) {
        // Referans trajektoriyi güncelle
        std::vector<double> ref_traj(N * n_states, 0.0);
        for (int k = 0; k < N; ++k) {
            // x pozisyonu artıyor
            double x_ref = x0[0] + (k + 1) * dt * 1.0;
            ref_traj[k * n_states + 0] = x_ref;

            // y pozisyonu sinüs dalgası şeklinde
            double y_ref = amplitude * sin(frequency * x_ref);
            ref_traj[k * n_states + 1] = y_ref;

            // Theta açısı yolun eğimine göre ayarlanıyor
            double dx = 1.0;  // x artışı sabit
            double dy = amplitude * frequency * cos(frequency * x_ref);  // y'nin türevi
            double theta_ref = atan2(dy, dx);
            ref_traj[k * n_states + 2] = theta_ref;

            // Tam referans yolunu biriktir
            if (k == 0) {
                if (mpc_iter == 0) {
                    ref_traj_full.push_back({x_ref, y_ref});
                } else {
                    ref_traj_full.push_back({x_ref, y_ref});
                }
            }
        }

        // Parametreleri güncelle
        std::vector<double> p_vals;
        p_vals.insert(p_vals.end(), x0.begin(), x0.end());
        p_vals.insert(p_vals.end(), ref_traj.begin(), ref_traj.end());

        // Başlangıç değerleri
        std::vector<double> x_init;
        x_init.insert(x_init.end(), u0.begin(), u0.end());
        x_init.insert(x_init.end(), x_mpc.begin(), x_mpc.end());

        // Kısıt sınırları
        int nlp_g = g_concat.size().first;
        std::vector<double> lbg(nlp_g, 0.0);
        std::vector<double> ubg(nlp_g, 0.0);

        // Optimizasyon değişkenlerinin sınırları
        int nlp_x = OPT_variables.size().first;
        std::vector<double> lbx(nlp_x, -1e20);
        std::vector<double> ubx(nlp_x, 1e20);

        // Problemi çöz
        DMDict arg = {{"x0", x_init}, {"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}, {"p", p_vals}};
        DMDict sol = solver(arg);

        // Çözümleri al
        std::vector<double> sol_vector = std::vector<double>(static_cast<std::vector<double>>(sol.at("x")));
        std::vector<double> sol_u(sol_vector.begin(), sol_vector.begin() + N * n_controls);
        std::vector<double> sol_x(sol_vector.begin() + N * n_controls, sol_vector.end());

        // İlk kontrol girdisini uygula
        std::vector<double> u_applied(sol_u.begin(), sol_u.begin() + n_controls);
        u_cl.push_back(u_applied);

        // Sistemi güncelle
        DMDict f_value = f({x0, u_applied});
        std::vector<double> f_vec = static_cast<std::vector<double>>(f_value.at(0));
        for (int i = 0; i < n_states; ++i) {
            x0[i] += dt * f_vec[i];
        }
        t += dt;
        xx.push_back(x0);

        // Kontrol ve durum tahminlerini güncelle
        u0.erase(u0.begin(), u0.begin() + n_controls);
        u0.insert(u0.end(), sol_u.end() - n_controls, sol_u.end());

        x_mpc.erase(x_mpc.begin(), x_mpc.begin() + n_states);
        x_mpc.insert(x_mpc.end(), sol_x.end() - n_states, sol_x.end());

        mpc_iter += 1;
    }

    // Sonuçları dosyaya yazalım
    std::ofstream outfile("mpc_result.csv");
    outfile << "x,y,ref_x,ref_y\n";
    for (size_t i = 0; i < xx.size(); ++i) {
        outfile << xx[i][0] << "," << xx[i][1] << "," << ref_traj_full[i][0] << "," << ref_traj_full[i][1] << "\n";
    }
    outfile.close();

    std::cout << "MPC Simülasyonu Tamamlandı. Sonuçlar 'mpc_result.csv' dosyasına kaydedildi." << std::endl;

    return 0;
}
