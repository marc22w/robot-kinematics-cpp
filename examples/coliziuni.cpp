#include <iomanip>
#include <iostream>

#include "kinematics/collision.hpp"
#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

namespace {

constexpr double kGrad = 3.14159265358979323846 / 180.0;

void raport(const char* nume, const kin::RezultatColiziune& r) {
    std::cout << "  " << std::left << std::setw(34) << nume << std::right
              << (r.coliziune ? "COLIZIUNE  " : "liber      ") << std::fixed << std::setprecision(1)
              << "distanta minima " << std::setw(7) << r.distanta * 1000 << " mm  ("
              << r.perecheCritica << ")\n";
}

void raportTraiectorie(const char* nume, const kin::Robot& robot, const kin::Trajectory& traj,
                       const kin::Scena& scena) {
    const int k = kin::primaColiziune(robot, traj, scena);
    std::cout << "  " << std::left << std::setw(34) << nume << std::right;
    if (k < 0) {
        std::cout << "fara coliziuni pe " << traj.size() << " puncte\n";
    } else {
        const kin::RezultatColiziune r = kin::verificaColiziuni(robot, traj[k].q, scena);
        std::cout << "COLIZIUNE la t = " << std::setprecision(2) << traj[k].t << " s ("
                  << r.perecheCritica << ")\n";
    }
}

}  // namespace

int main() {
    const kin::Robot robot = kin::Robot::ur5();
    kin::Joints acasa, pliat;
    acasa << 0, -90, 90, -90, -90, 0;
    pliat << 0, -90, 170, -90, -90, 0;
    acasa *= kGrad;
    pliat *= kGrad;

    kin::Scena masa;
    masa.planuri.push_back({"masa", Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)});
    kin::Scena cuStalp = masa;
    cuStalp.obstacole.push_back(
        {"stalp", {Eigen::Vector3d(-0.40, -0.32, 0.0), Eigen::Vector3d(-0.40, -0.32, 1.0), 0.03}});

    std::cout << "POZITII\n";
    raport("Pozitia de lucru", kin::verificaColiziuni(robot, acasa, masa));
    raport("Cot pliat la 170 de grade", kin::verificaColiziuni(robot, pliat, masa));

    // aceeasi miscare LIN ca in demo
    const Eigen::Isometry3d start = robot.forward(acasa);
    Eigen::Isometry3d capat = start;
    capat.translation() << -0.300, -0.400, 0.250;
    capat.linear() = Eigen::AngleAxisd(30.0 * kGrad, Eigen::Vector3d::UnitZ()) * start.linear();
    const kin::CartesianResult lin = kin::linearCartesian(robot, acasa, capat, 3.0, 0.01);

    Eigen::Isometry3d jos = start;
    jos.translation().z() = 0.030;
    const kin::CartesianResult coborare = kin::linearCartesian(robot, acasa, jos, 2.0, 0.01);

    std::cout << "\nTRAIECTORII\n";
    raportTraiectorie("LIN, doar masa", robot, lin.traj, masa);
    raportTraiectorie("LIN, cu stalp (D60) pe traseu", robot, lin.traj, cuStalp);
    raportTraiectorie("Coborare LIN pana la 30 mm de masa", robot, coborare.traj, masa);
    return 0;
}
