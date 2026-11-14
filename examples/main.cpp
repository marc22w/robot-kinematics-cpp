#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

namespace {

// Traseul sculei (pozitia flansei, in mm) de-a lungul unei traiectorii
void scrieCale(const std::string& fisier, const kin::Robot& robot, const kin::Trajectory& traj) {
    std::ofstream out(fisier);
    out.precision(10);
    out << "t,x,y,z\n";
    for (const kin::TrajPoint& p : traj) {
        const Eigen::Vector3d x = robot.forward(p.q).translation() * 1000.0;
        out << p.t << "," << x.x() << "," << x.y() << "," << x.z() << "\n";
    }
}

}  // namespace

int main() {
    constexpr double grad = 3.14159265358979323846 / 180.0;
    const kin::Robot robot = kin::Robot::ur5();
    std::cout << std::fixed << std::setprecision(2);

    kin::Joints acasa;
    acasa << 0, -90, 90, -90, -90, 0;
    acasa *= grad;

    // --- cinematica inversa: scula in jos, deasupra unui punct de pe masa ---
    Eigen::Isometry3d tinta = Eigen::Isometry3d::Identity();
    tinta.linear() << 0, 1, 0,
                      1, 0, 0,
                      0, 0, -1;
    tinta.translation() << -0.400, -0.200, 0.300;
    const kin::IKResult ik = robot.inverse(tinta, acasa);
    std::cout << "CINEMATICA INVERSA: tinta (-400, -200, 300) mm, scula in jos\n  Unghiuri (grade):";
    for (int i = 0; i < kin::kAxe; ++i) std::cout << "  A" << i + 1 << "=" << ik.q[i] / grad;
    std::cout << "\n\n";

    // --- miscare PTP la limitele de viteza si acceleratie ---
    kin::Joints q1;
    q1 << 120, -45, 30, -120, -90, 45;
    q1 *= grad;
    const kin::Joints vmax = kin::Joints::Constant(180.0 * grad);  // 180 grade/s
    const kin::Joints amax = kin::Joints::Constant(4.0);           // rad/s^2
    const double T = kin::durataMinima(acasa, q1, vmax, amax);
    const kin::Trajectory ptp = kin::quinticJoint(acasa, q1, T, 0.01);
    kin::scrieCSV("traiectorie_ptp.csv", ptp);
    std::cout << "MISCARE PTP (articulatii)\n  Durata minima: " << T << " s, "
              << ptp.size() << " puncte -> traiectorie_ptp.csv\n\n";

    // --- miscare LIN: scula pe o dreapta, cu rotire de 30 de grade ---
    const Eigen::Isometry3d start = robot.forward(acasa);
    Eigen::Isometry3d capat = start;
    capat.translation() << -0.300, -0.400, 0.250;
    capat.linear() = Eigen::AngleAxisd(30.0 * grad, Eigen::Vector3d::UnitZ()) * start.linear();
    const kin::CartesianResult lin = kin::linearCartesian(robot, acasa, capat, 3.0, 0.01);
    std::cout << "MISCARE LIN (linie dreapta)\n  Reusit: " << (lin.ok ? "da" : "nu")
              << ", " << lin.traj.size() << " puncte -> traiectorie_lin.csv\n";
    if (!lin.ok) return 1;
    kin::scrieCSV("traiectorie_lin.csv", lin.traj);

    // --- comparatie: acelasi punct final, atins cu o miscare PTP ---
    const kin::Trajectory ptpSpreCapat = kin::quinticJoint(acasa, lin.traj.back().q, 3.0, 0.01);
    scrieCale("cale_lin.csv", robot, lin.traj);
    scrieCale("cale_ptp.csv", robot, ptpSpreCapat);
    std::cout << "\nTraseele sculei pentru comparatie -> cale_lin.csv, cale_ptp.csv\n";
    return 0;
}
