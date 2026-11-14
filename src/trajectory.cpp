#include "kinematics/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "kinematics/ur_analytic.hpp"

namespace kin {

namespace {

int numarPasi(double durata, double dt) {
    if (durata <= 0.0 || dt <= 0.0) {
        throw std::invalid_argument("durata si dt trebuie sa fie pozitive");
    }
    return std::max(1, static_cast<int>(std::ceil(durata / dt - 1e-9)));
}

// Unghiurile articulatiilor in punctele dreptei start -> tinta, pentru parametrii s[k] in [0, 1].
// Se opreste la primul punct inaccesibil sau la un salt prea mare (singularitate).
bool traseuDrept(const Robot& robot, const Joints& qStart, const Eigen::Isometry3d& tinta,
                 const std::vector<double>& s, const LinOptions& opt,
                 std::vector<Joints>& puncte, int& esuatLa, std::string& motiv) {
    const Eigen::Isometry3d start = robot.forward(qStart);
    const Eigen::Quaterniond r0(start.rotation());
    const Eigen::Quaterniond r1(tinta.rotation());

    IKOptions ikOpt;
    ikOpt.restarts = 0;   // fara reporniri: robotul nu are voie sa sara in alta configuratie

    puncte.clear();
    puncte.reserve(s.size());
    Joints q = qStart;
    for (size_t k = 0; k < s.size(); ++k) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = r0.slerp(s[k], r1).toRotationMatrix();
        T.translation() = start.translation() + s[k] * (tinta.translation() - start.translation());

        Joints q_nou;
        if (opt.solver == SolverIK::AnaliticUR5) {
            const auto sol = celMaiApropiat(inverseUR5(T), q, robot.limits());
            if (!sol) {
                esuatLa = static_cast<int>(k);
                motiv = "tinta inaccesibila";
                return false;
            }
            q_nou = *sol;
        } else {
            const IKResult ik = robot.inverse(T, q, ikOpt);
            if (!ik.converged) {
                esuatLa = static_cast<int>(k);
                motiv = "tinta inaccesibila";
                return false;
            }
            // varianta (+-360 grade) cea mai apropiata de punctul anterior
            Joints dq = ik.q - q;
            for (int i = 0; i < kAxe; ++i) dq[i] = std::atan2(std::sin(dq[i]), std::cos(dq[i]));
            q_nou = q + dq;
        }

        // continuitate: un salt mare intre puncte vecine inseamna trecerea printr-o singularitate
        int axa = 0;
        const double salt = (q_nou - q).cwiseAbs().maxCoeff(&axa);
        if (salt > opt.saltMaxim) {
            std::ostringstream m;
            m << std::fixed << std::setprecision(1) << "salt de " << salt * 180.0 / 3.14159265358979
              << " grade pe axa " << axa + 1 << " (singularitate)";
            esuatLa = static_cast<int>(k);
            motiv = m.str();
            return false;
        }
        q = q_nou;
        puncte.push_back(q);
    }
    return true;
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
                                const Eigen::Isometry3d& tinta, double durata, double dt,
                                const LinOptions& opt) {
    const int n = numarPasi(durata, dt);
    std::vector<double> s(n + 1);
    for (int k = 0; k <= n; ++k) s[k] = profil(static_cast<double>(k) / n);

    CartesianResult res;
    std::vector<Joints> q;
    const bool ok = traseuDrept(robot, qStart, tinta, s, opt, q, res.esuatLa, res.motiv);
    res.traj.resize(q.size());
    for (size_t k = 0; k < q.size(); ++k) {
        res.traj[k].t = static_cast<double>(k) / n * durata;
        res.traj[k].q = q[k];
    }
    if (!ok) return res;

    // vitezele si acceleratiile articulatiilor, prin diferente finite centrale
    const double h = durata / n;
    for (int k = 1; k < n; ++k) {
        res.traj[k].qd = (q[k + 1] - q[k - 1]) / (2.0 * h);
        res.traj[k].qdd = (q[k + 1] - 2.0 * q[k] + q[k - 1]) / (h * h);
    }
    res.ok = true;
    return res;
}

CartesianResult linearCartesianViteza(const Robot& robot, const Joints& qStart,
                                      const Eigen::Isometry3d& tinta, const LinViteza& v,
                                      const LinOptions& opt) {
    const double L = (tinta.translation() - robot.forward(qStart).translation()).norm();
    if (L < 1e-9) throw std::invalid_argument("miscarea LIN are nevoie de o deplasare a sculei");
    if (v.vScula <= 0.0 || v.aScula <= 0.0 || v.pas <= 0.0) {
        throw std::invalid_argument("vScula, aScula si pas trebuie sa fie pozitive");
    }
    const int n = std::max(2, static_cast<int>(std::ceil(L / v.pas - 1e-9)));
    const double dx = L / n;   // m, distanta reala dintre doua puncte

    // 1. traseul geometric, la pas constant pe dreapta
    std::vector<double> s(n + 1);
    for (int k = 0; k <= n; ++k) s[k] = static_cast<double>(k) / n;
    CartesianResult res;
    std::vector<Joints> q;
    if (!traseuDrept(robot, qStart, tinta, s, opt, q, res.esuatLa, res.motiv)) {
        for (const Joints& qk : q) res.traj.push_back(TrajPoint{0.0, qk});
        return res;
    }

    // 2. cat de repede variaza fiecare unghi pe metrul de traseu (rad/m)
    std::vector<Joints> g(n + 1);
    g[0] = (q[1] - q[0]) / dx;
    g[n] = (q[n] - q[n - 1]) / dx;
    for (int k = 1; k < n; ++k) g[k] = (q[k + 1] - q[k - 1]) / (2.0 * dx);

    // 3. viteza maxima permisa a sculei in fiecare punct
    std::vector<double> vlim(n + 1);
    for (int k = 0; k <= n; ++k) {
        double lim = v.vScula;
        for (int i = 0; i < kAxe; ++i) {
            const double gi = std::abs(g[k][i]);
            if (gi > 1e-12) lim = std::min(lim, v.vmax[i] / gi);
        }
        vlim[k] = lim;
    }

    // 4. profilul de viteza: pornire si oprire din repaus, acceleratie cel mult aScula
    std::vector<double> vit = vlim;
    vit[0] = 0.0;
    vit[n] = 0.0;
    for (int k = 0; k < n; ++k) {   // inainte: cat poate accelera
        vit[k + 1] = std::min(vit[k + 1], std::sqrt(vit[k] * vit[k] + 2.0 * v.aScula * dx));
    }
    for (int k = n; k > 0; --k) {   // inapoi: ca sa poata frana la timp
        vit[k - 1] = std::min(vit[k - 1], std::sqrt(vit[k] * vit[k] + 2.0 * v.aScula * dx));
    }

    // 5. timpul in fiecare punct si vitezele/acceleratiile articulatiilor
    res.traj.resize(n + 1);
    double t = 0.0;
    for (int k = 0; k <= n; ++k) {
        if (k > 0) t += 2.0 * dx / (vit[k - 1] + vit[k]);
        res.traj[k].t = t;
        res.traj[k].q = q[k];
        res.traj[k].qd = g[k] * vit[k];
    }
    for (int k = 1; k < n; ++k) {
        res.traj[k].qdd = (res.traj[k + 1].qd - res.traj[k - 1].qd) /
                          (res.traj[k + 1].t - res.traj[k - 1].t);
    }

    res.vitezaMinima = *std::min_element(vlim.begin(), vlim.end());
    res.puncteLimitate = static_cast<int>(
        std::count_if(vlim.begin(), vlim.end(), [&](double x) { return x < v.vScula - 1e-12; }));
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
