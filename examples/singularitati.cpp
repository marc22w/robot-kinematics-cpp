#include <iomanip>
#include <iostream>
#include <string>

#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

namespace {

constexpr double kGrad = 3.14159265358979323846 / 180.0;

kin::Joints grade(double a1, double a2, double a3, double a4, double a5, double a6) {
    kin::Joints q;
    q << a1, a2, a3, a4, a5, a6;
    return q * kGrad;
}

void raport(const kin::Robot& r, const std::string& nume, const kin::Joints& q) {
    const kin::SingularitatiUR s = kin::singularitatiUR5(q);
    std::cout << std::left << std::setw(26) << nume << std::right << std::fixed
              << std::setprecision(3) << " det J=" << r.manipulability(q)
              << "  incheietura=" << s.incheietura << "  cot=" << s.cot
              << "  umar=" << std::setw(7) << std::setprecision(2) << s.umar * 1000 << " mm"
              << (s.aproape() ? "   <-- SINGULARITATE" : "") << "\n";
}

}  // namespace

int main() {
    const kin::Robot robot = kin::Robot::ur5();
    std::cout << "ANALIZA SINGULARITATILOR UR5\n";
    raport(robot, "Pozitia de lucru", grade(0, -90, 90, -90, -90, 0));
    raport(robot, "Incheietura (A5 = 0)", grade(0, -90, 90, -90, 0, 0));
    raport(robot, "Cot, brat intins (A3 = 0)", grade(0, -45, 0, -45, -90, 0));
    raport(robot, "Umar", grade(0, -90, 13.96, 166.04, -90, 0));

    // miscare LIN care trece prin singularitatea incheieturii
    std::cout << "\nMISCARE LIN PRIN SINGULARITATEA INCHEIETURII (A5: -10 -> +10 grade)\n";
    const kin::Joints q0 = grade(0, -90, 90, -90, -10, 0);
    const Eigen::Isometry3d tinta = robot.forward(grade(0, -90, 90, -90, 10, 0));
    for (const kin::SolverIK solver : {kin::SolverIK::Numeric, kin::SolverIK::AnaliticUR5}) {
        kin::LinOptions opt;
        opt.solver = solver;
        const kin::CartesianResult r = kin::linearCartesian(robot, q0, tinta, 2.0, 0.01, opt);
        std::cout << "  " << (solver == kin::SolverIK::Numeric ? "Numeric:  " : "Analitic: ");
        if (r.ok) {
            std::cout << "reusit\n";
        } else {
            std::cout << "oprit la t = " << std::setprecision(2) << r.esuatLa * 0.01 << " s, "
                      << r.motiv << "\n";
        }
    }
    // miscare LIN pe langa axa 1, cu si fara limitarea vitezei articulatiilor
    std::cout << "\nVITEZA LIMITATA: LIN de 700 mm pe langa axa 1, scula la 0.5 m/s\n";
    const kin::Joints qb = grade(3.38, -107.07, 116.33, -99.26, -90, 3.38);
    Eigen::Isometry3d capat = robot.forward(qb);
    capat.translation().x() += 0.70;
    kin::LinViteza v;
    v.vScula = 0.5;
    v.aScula = 1.5;
    kin::LinViteza faraLimite = v;
    faraLimite.vmax = kin::Joints::Constant(1e9);

    auto vitezaMaxima = [](const kin::Trajectory& traj, int& axa) {
        double m = 0.0;
        for (const kin::TrajPoint& p : traj) {
            int i = 0;
            const double x = p.qd.cwiseAbs().maxCoeff(&i);
            if (x > m) { m = x; axa = i + 1; }
        }
        return m / kGrad;
    };
    const kin::CartesianResult liber = kin::linearCartesianViteza(robot, qb, capat, faraLimite);
    const kin::CartesianResult limitat = kin::linearCartesianViteza(robot, qb, capat, v);
    int axaL = 0, axaV = 0;
    const double vL = vitezaMaxima(liber.traj, axaL), vV = vitezaMaxima(limitat.traj, axaV);
    std::cout << std::setprecision(1)
              << "  Fara limitare: axa " << axaL << " ar ajunge la " << vL
              << " grade/s (limita 180), durata " << std::setprecision(3) << liber.traj.back().t << " s\n"
              << std::setprecision(1)
              << "  Cu limitare:   scula incetineste pana la " << std::setprecision(3)
              << limitat.vitezaMinima << " m/s, durata " << limitat.traj.back().t
              << " s, viteza maxima pe articulatii " << std::setprecision(1) << vV << " grade/s\n";
    return 0;
}
