#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "kinematics/robot.hpp"
#include "kinematics/ur_analytic.hpp"

int main() {
    using Ceas = std::chrono::steady_clock;
    constexpr int N = 2000;
    constexpr double kPi = 3.14159265358979323846;
    const kin::Robot robot = kin::Robot::ur5();

    std::mt19937 gen(1);
    std::uniform_real_distribution<double> u(-kPi, kPi), zgomot(-0.1, 0.1);
    std::vector<kin::Joints> reale(N), apropiate(N);
    std::vector<Eigen::Isometry3d> tinte(N);
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < kin::kAxe; ++i) {
            reale[k][i] = u(gen);
            apropiate[k][i] = reale[k][i] + zgomot(gen);
        }
        tinte[k] = robot.forward(reale[k]);
    }

    auto masoara = [&](int repetari, auto&& functie) {
        const auto t0 = Ceas::now();
        for (int r = 0; r < repetari; ++r)
            for (int k = 0; k < N; ++k) functie(k);
        return std::chrono::duration<double, std::micro>(Ceas::now() - t0).count() /
               (static_cast<double>(N) * repetari);
    };

    double suma = 0.0;   // folosim rezultatul, ca compilatorul sa nu elimine calculul
    const double tDirecta = masoara(50, [&](int k) { suma += robot.forward(reale[k])(0, 3); });

    size_t solutii = 0;
    const double tAnalitic =
        masoara(5, [&](int k) { solutii += kin::inverseUR5(tinte[k]).size(); });

    long iteratiiA = 0;
    int convergenteA = 0, convergenteZ = 0;
    const double tApropiat = masoara(1, [&](int k) {
        const kin::IKResult r = robot.inverse(tinte[k], apropiate[k]);
        convergenteA += r.converged;
        iteratiiA += r.iterations;
    });
    const double tZero = masoara(1, [&](int k) {
        convergenteZ += robot.inverse(tinte[k], kin::Joints::Zero()).converged;
    });

    std::cout << std::fixed << std::setprecision(1)
              << "BENCHMARK UR5 (" << N << " tinte aleatoare)\n\n"
              << "  Cinematica directa:            " << std::setw(8) << tDirecta * 1000.0
              << " ns/calcul\n"
              << "  IK analitic, toate solutiile:  " << std::setw(8) << tAnalitic << " us/tinta"
              << "   (in medie " << static_cast<double>(solutii) / (N * 5) << " solutii)\n"
              << "  IK numeric, pornire apropiata: " << std::setw(8) << tApropiat << " us/tinta"
              << "   (" << 100.0 * convergenteA / N << "% reusite, in medie "
              << static_cast<double>(iteratiiA) / N << " iteratii)\n"
              << "  IK numeric, pornire din zero:  " << std::setw(8) << tZero << " us/tinta"
              << "   (" << 100.0 * convergenteZ / N << "% reusite)\n"
              << "\n  (control: " << std::setprecision(3) << suma << ")\n";
    return 0;
}
