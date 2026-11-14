#include <iomanip>
#include <iostream>

#include "kinematics/robot.hpp"

int main() {
    constexpr double grad = 3.14159265358979323846 / 180.0;
    const kin::Robot robot = kin::Robot::ur5();

    // pozitie clasica de lucru: scula orientata in jos
    kin::Joints q;
    q << 0, -90, 90, -90, -90, 0;
    q *= grad;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Pozitia fiecarei articulatii (mm):\n";
    const auto f = robot.frames(q);
    for (int i = 1; i <= kin::kAxe; ++i) {
        const Eigen::Vector3d p = f[i].translation() * 1000.0;
        std::cout << "  A" << i << ": x=" << std::setw(8) << p.x()
                  << "  y=" << std::setw(8) << p.y()
                  << "  z=" << std::setw(8) << p.z() << "\n";
    }

    std::cout << "\nOrientarea flansei (matrice de rotatie):\n"
              << robot.forward(q).rotation() << "\n";
    return 0;
}
