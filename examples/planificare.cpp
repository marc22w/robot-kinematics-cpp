#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>

#include "kinematics/collision.hpp"
#include "kinematics/planner.hpp"
#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

int main() {
    constexpr double kGrad = 3.14159265358979323846 / 180.0;
    const kin::Robot robot = kin::Robot::ur5();

    kin::Joints start, tinta, mijloc;
    start << -60, -90, 100, -100, -90, 0;
    tinta << 60, -90, 100, -100, -90, 0;
    mijloc << 0, -90, 100, -100, -90, 0;
    start *= kGrad;
    tinta *= kGrad;
    mijloc *= kGrad;

    const Eigen::Vector3d p = robot.forward(mijloc).translation();
    kin::Scena scena;
    scena.planuri.push_back({"masa", Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)});
    scena.obstacole.push_back(
        {"stalp", {Eigen::Vector3d(p.x(), p.y(), 0.0), Eigen::Vector3d(p.x(), p.y(), 0.55), 0.04}});

    const kin::Joints vmax = kin::Joints::Constant(180.0 * kGrad);
    const kin::Joints amax = kin::Joints::Constant(4.0);

    std::cout << std::fixed << std::setprecision(2)
              << "PLANIFICARE: rotire pe A1 de la -60 la +60 de grade, stalp D80 x 550 mm in drum\n\n";

    const kin::Trajectory direct = kin::quinticJoint(start, tinta, kin::durataMinima(start, tinta, vmax, amax), 0.01);
    const int k = kin::primaColiziune(robot, direct, scena);
    if (k >= 0) {
        const kin::RezultatColiziune c = kin::verificaColiziuni(robot, direct[k].q, scena);
        std::cout << "  PTP direct: COLIZIUNE la t = " << direct[k].t << " s (" << c.perecheCritica << ")\n";
    }

    const auto t0 = std::chrono::steady_clock::now();
    const kin::RezultatRRT res = kin::planificaRRT(robot, start, tinta, scena);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!res.gasit) {
        std::cout << "  RRT: nu am gasit drum (" << res.motiv << ")\n";
        return 1;
    }
    const kin::Trajectory traj = kin::traiectorieDinCale(res.cale, vmax, amax, 0.01);
    double distMin = 1e9;
    for (const kin::TrajPoint& pt : traj) {
        distMin = std::min(distMin, kin::verificaColiziuni(robot, pt.q, scena).distanta);
    }
    std::cout << "  RRT-Connect: drum gasit in " << ms << " ms (" << res.iteratii << " iteratii, " << res.noduri
              << " noduri)\n"
              << "  Puncte de trecere: " << res.cale.size() << ", lungime " << kin::lungimeCale(res.cale) / kGrad
              << " grade (direct: " << kin::lungimeCale({start, tinta}) / kGrad << ")\n"
              << "  Durata: " << traj.back().t << " s (direct: " << direct.back().t << " s)\n"
              << "  Cea mai mica distanta fata de obstacole: " << std::setprecision(1) << distMin * 1000 << " mm\n";
    return 0;
}
