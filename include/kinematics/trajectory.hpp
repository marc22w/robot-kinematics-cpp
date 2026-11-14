#pragma once

#include <string>
#include <vector>

#include "kinematics/robot.hpp"

namespace kin {

struct TrajPoint {
    double t = 0.0;               // s
    Joints q = Joints::Zero();    // rad
    Joints qd = Joints::Zero();   // rad/s
    Joints qdd = Joints::Zero();  // rad/s^2
};
using Trajectory = std::vector<TrajPoint>;

// Profil "minimum jerk" pe intervalul normalizat u in [0, 1]:
// s(0) = 0, s(1) = 1, viteza si acceleratia zero la ambele capete
double profil(double u);
double profilViteza(double u);       // ds/du
double profilAcceleratie(double u);  // d2s/du2

// Miscare in spatiul articulatiilor, polinom de gradul 5 pe fiecare articulatie
Trajectory quinticJoint(const Joints& q0, const Joints& q1, double durata, double dt);

// Cea mai scurta durata la care nicio articulatie nu depaseste vmax (rad/s) si amax (rad/s^2)
double durataMinima(const Joints& q0, const Joints& q1, const Joints& vmax, const Joints& amax);

// Miscare liniara a sculei (tip LIN): pozitia pe o dreapta, orientarea interpolata lin
struct CartesianResult {
    Trajectory traj;
    bool ok = false;
    int esuatLa = -1;   // indexul punctului pe care robotul nu il mai poate atinge
};
CartesianResult linearCartesian(const Robot& robot, const Joints& qStart,
                                const Eigen::Isometry3d& tinta, double durata, double dt);

// Salveaza traiectoria in CSV: t, q1..q6, qd1..qd6, qdd1..qdd6
void scrieCSV(const std::string& fisier, const Trajectory& traj);

}  // namespace kin
