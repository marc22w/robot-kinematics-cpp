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

    auto masoara = [&](auto&& functie) {
        const auto t0 = Ceas::now();
        for (int k = 0; k < N; ++k) functie(k);
        return std::chrono::duration<double, std::micro>(Ceas::now() - t0).count() / N;
    };

    size_t solutii = 0;
    const double tAnalitic = masoara([&](int k) { solutii += kin::inverseUR5(tinte[k]).size(); });

    int convergenteA = 0, convergenteZ = 0;
    const double tApropiat = masoara([&](int k) {
        convergenteA += robot.inverse(tinte[k], apropiate[k]).converged;
    });
    const double tZero = masoara([&](int k) {
        convergenteZ += robot.inverse(tinte[k], kin::Joints::Zero()).converged;
    });

    std::cout << std::fixed << std::setprecision(1)
              << "BENCHMARK CINEMATICA INVERSA UR5 (" << N << " tinte aleatoare)\n\n"
              << "  Analitic, toate solutiile:     " << std::setw(8) << tAnalitic << " us/tinta"
              << "   (in medie " << static_cast<double>(solutii) / N << " solutii)\n"
              << "  Numeric, pornire apropiata:    " << std::setw(8) << tApropiat << " us/tinta"
              << "   (" << 100.0 * convergenteA / N << "% reusite)\n"
              << "  Numeric, pornire din zero:     " << std::setw(8) << tZero << " us/tinta"
              << "   (" << 100.0 * convergenteZ / N << "% reusite)\n\n"
              << "  Analiticul e de " << tApropiat / tAnalitic << "x mai rapid decat numericul cu"
              << " pornire apropiata si de " << tZero / tAnalitic << "x fata de pornirea din zero.\n";
    return 0;
}
