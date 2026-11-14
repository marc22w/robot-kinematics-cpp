#include <iomanip>
#include <iostream>

#include "kinematics/robot.hpp"

int main() {
    constexpr double grad = 3.14159265358979323846 / 180.0;
    const kin::Robot robot = kin::Robot::ur5();
    std::cout << std::fixed << std::setprecision(2);

    // --- cinematica directa ---
    kin::Joints q;
    q << 0, -90, 90, -90, -90, 0;
    q *= grad;
    const Eigen::Vector3d p = robot.forward(q).translation() * 1000.0;
    std::cout << "CINEMATICA DIRECTA\n"
              << "  Flansa la (mm): x=" << p.x() << "  y=" << p.y() << "  z=" << p.z() << "\n\n";

    // --- cinematica inversa: scula in jos, deasupra unui punct de pe masa ---
    Eigen::Isometry3d tinta = Eigen::Isometry3d::Identity();
    tinta.linear() << 0, 1, 0,
                      1, 0, 0,
                      0, 0, -1;
    tinta.translation() << -0.400, -0.200, 0.300;   // m

    const kin::IKResult res = robot.inverse(tinta, q);
    std::cout << "CINEMATICA INVERSA, tinta (mm): x=-400  y=-200  z=300, scula in jos\n"
              << "  Gasit: " << (res.converged ? "da" : "nu")
              << " in " << res.iterations << " iteratii\n  Unghiuri (grade):";
    for (int i = 0; i < kin::kAxe; ++i) std::cout << "  A" << i + 1 << "=" << res.q[i] / grad;
    std::cout << std::scientific << std::setprecision(1)
              << "\n  Eroare de pozitie: " << res.posError * 1000.0 << " mm\n";
    return 0;
}
