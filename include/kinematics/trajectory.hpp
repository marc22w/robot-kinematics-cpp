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

// Cum se rezolva cinematica inversa in fiecare punct al unei miscari LIN
enum class SolverIK {
    Numeric,       // Levenberg-Marquardt, pornind din punctul anterior (orice robot)
    AnaliticUR5,   // formulele exacte UR5, solutia cea mai apropiata de punctul anterior
};

struct LinOptions {
    SolverIK solver = SolverIK::Numeric;
    double saltMaxim = 0.2;   // rad: cat poate sari o articulatie intre doua puncte vecine
};

// Miscare liniara a sculei (tip LIN): pozitia pe o dreapta, orientarea interpolata lin
struct CartesianResult {
    Trajectory traj;
    bool ok = false;
    int esuatLa = -1;     // indexul primului punct care nu poate fi atins continuu
    std::string motiv;    // de ce s-a oprit miscarea
    double vitezaMinima = 0.0;  // m/s: cea mai mica viteza impusa sculei de limitele articulatiilor
    int puncteLimitate = 0;     // in cate puncte a trebuit incetinita scula
};
CartesianResult linearCartesian(const Robot& robot, const Joints& qStart,
                                const Eigen::Isometry3d& tinta, double durata, double dt,
                                const LinOptions& opt = LinOptions{});

// Parametrii unei miscari LIN cu viteza controlata
struct LinViteza {
    double vScula = 0.25;      // m/s: viteza nominala a sculei
    double aScula = 1.0;       // m/s^2: acceleratia maxima a sculei pe traseu
    Joints vmax = Joints::Constant(3.14159265358979323846);   // rad/s, pe fiecare articulatie
    double pas = 0.001;        // m: distanta dintre doua puncte ale traseului
};

// Miscare LIN cu profil de viteza trapezoidal: scula merge cu vScula, iar acolo unde o
// articulatie ar depasi vmax (de ex. langa o singularitate) scula este incetinita automat.
// Durata rezulta din profil.
CartesianResult linearCartesianViteza(const Robot& robot, const Joints& qStart,
                                      const Eigen::Isometry3d& tinta, const LinViteza& v,
                                      const LinOptions& opt = LinOptions{});

// Salveaza traiectoria in CSV: t, q1..q6, qd1..qd6, qdd1..qdd6
void scrieCSV(const std::string& fisier, const Trajectory& traj);

}  // namespace kin
