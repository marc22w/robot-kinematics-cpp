#include "kinematics/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace kin {

namespace {

int numarPasi(double durata, double dt) {
    if (durata <= 0.0 || dt <= 0.0) {
        throw std::invalid_argument("durata si dt trebuie sa fie pozitive");
    }
    return std::max(1, static_cast<int>(std::ceil(durata / dt - 1e-9)));
}

}  // namespace

double profil(double u) { return u * u * u * (10.0 + u * (-15.0 + 6.0 * u)); }
double profilViteza(double u) { return 30.0 * u * u * (1.0 - u) * (1.0 - u); }
double profilAcceleratie(double u) { return 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u); }

Trajectory quinticJoint(const Joints& q0, const Joints& q1, double durata, double dt) {
    const int n = numarPasi(durata, dt);
    const Joints delta = q1 - q0;
    Trajectory traj;
    traj.reserve(n + 1);
    for (int k = 0; k <= n; ++k) {
        const double u = static_cast<double>(k) / n;
        TrajPoint p;
        p.t = u * durata;
        p.q = q0 + profil(u) * delta;
        p.qd = profilViteza(u) / durata * delta;
        p.qdd = profilAcceleratie(u) / (durata * durata) * delta;
        traj.push_back(p);
    }
    return traj;
}

double durataMinima(const Joints& q0, const Joints& q1, const Joints& vmax, const Joints& amax) {
    const double vVarf = 1.875;                  // max ds/du, la u = 0.5
    const double aVarf = 10.0 / std::sqrt(3.0);  // max |d2s/du2|
    double T = 0.0;
    for (int i = 0; i < kAxe; ++i) {
        const double d = std::abs(q1[i] - q0[i]);
        if (d == 0.0) continue;
        T = std::max(T, vVarf * d / vmax[i]);
        T = std::max(T, std::sqrt(aVarf * d / amax[i]));
    }
    return T;
}

CartesianResult linearCartesian(const Robot& robot, const Joints& qStart,
                                const Eigen::Isometry3d& tinta, double durata, double dt) {
    const int n = numarPasi(durata, dt);
    const Eigen::Isometry3d start = robot.forward(qStart);
    const Eigen::Quaterniond r0(start.rotation());
    const Eigen::Quaterniond r1(tinta.rotation());

    IKOptions opt;
    opt.restarts = 0;   // fara reporniri: robotul nu are voie sa sara in alta configuratie

    CartesianResult res;
    res.traj.reserve(n + 1);
    Joints q = qStart;
    for (int k = 0; k <= n; ++k) {
        const double u = static_cast<double>(k) / n;
        const double s = profil(u);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = r0.slerp(s, r1).toRotationMatrix();
        T.translation() = start.translation() + s * (tinta.translation() - start.translation());

        const IKResult ik = robot.inverse(T, q, opt);
        if (!ik.converged) {
            res.esuatLa = k;
            return res;
        }
        // pastram continuitatea: fara salturi de 360 de grade intre puncte vecine
        Joints dq = ik.q - q;
        for (int i = 0; i < kAxe; ++i) dq[i] = std::atan2(std::sin(dq[i]), std::cos(dq[i]));
        q += dq;

        TrajPoint p;
        p.t = u * durata;
        p.q = q;
        res.traj.push_back(p);
    }

    // vitezele si acceleratiile articulatiilor, prin diferente finite centrale
    const double h = durata / n;
    for (int k = 1; k < n; ++k) {
        const Joints& a = res.traj[k - 1].q;
        const Joints& b = res.traj[k].q;
        const Joints& c = res.traj[k + 1].q;
        res.traj[k].qd = (c - a) / (2.0 * h);
        res.traj[k].qdd = (c - 2.0 * b + a) / (h * h);
    }
    res.ok = true;
    return res;
}

void scrieCSV(const std::string& fisier, const Trajectory& traj) {
    std::ofstream out(fisier);
    if (!out) throw std::runtime_error("nu pot scrie fisierul " + fisier);
    out.precision(10);
    out << "t";
    for (const char* nume : {"q", "qd", "qdd"}) {
        for (int i = 1; i <= kAxe; ++i) out << "," << nume << i;
    }
    out << "\n";
    for (const TrajPoint& p : traj) {
        out << p.t;
        for (int i = 0; i < kAxe; ++i) out << "," << p.q[i];
        for (int i = 0; i < kAxe; ++i) out << "," << p.qd[i];
        for (int i = 0; i < kAxe; ++i) out << "," << p.qdd[i];
        out << "\n";
    }
}

}  // namespace kin
