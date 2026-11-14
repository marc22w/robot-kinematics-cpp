#include "kinematics/ur_analytic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kin {

namespace {

constexpr double kPi = 3.14159265358979323846;

// parametrii DH UR5 (aceiasi ca in Robot::ur5)
constexpr double d1 = 0.089159, a2 = -0.425, a3 = -0.39225;
constexpr double d4 = 0.10915, d5 = 0.09465, d6 = 0.0823;
const DH kDH[kAxe] = {
    {0.0, kPi / 2, d1}, {a2, 0.0, 0.0}, {a3, 0.0, 0.0},
    {0.0, kPi / 2, d4}, {0.0, -kPi / 2, d5}, {0.0, 0.0, d6},
};

double normal(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

std::vector<Joints> inverseUR5(const Eigen::Isometry3d& T06) {
    std::vector<Joints> solutii;

    // 1. A1, din centrul incheieturii (punctul de pe axa 5, la d6 in spatele flansei)
    const Eigen::Vector3d p05 = T06 * Eigen::Vector3d(0.0, 0.0, -d6);
    const double r = std::hypot(p05.x(), p05.y());
    if (r < d4) return solutii;   // incheietura prea aproape de axa 1
    const double phi = std::atan2(p05.y(), p05.x());
    const double psi = std::acos(d4 / r);

    const Eigen::Vector3d p06 = T06.translation();
    const Eigen::Matrix4d M = T06.inverse().matrix();

    for (const double t1 : {phi + psi + kPi / 2, phi - psi + kPi / 2}) {
        const double s1 = std::sin(t1), c1 = std::cos(t1);

        // 2. A5, din pozitia flansei proiectata pe directia axei 1
        double c5 = (p06.x() * s1 - p06.y() * c1 - d4) / d6;
        if (std::abs(c5) > 1.0 + 1e-12) continue;
        c5 = std::clamp(c5, -1.0, 1.0);

        for (const double t5 : {std::acos(c5), -std::acos(c5)}) {
            // 3. A6, din orientare
            const double s5 = std::sin(t5);
            double t6 = 0.0;
            if (std::abs(s5) > 1e-10) {
                t6 = std::atan2((-M(1, 0) * s1 + M(1, 1) * c1) / s5,
                                (M(0, 0) * s1 - M(0, 1) * c1) / s5);
            }

            // 4. A2, A3: triunghiul brat + antebrat, in sistemul articulatiei 1
            const Eigen::Isometry3d T14 = dhTransform(kDH[0], t1).inverse() * T06 *
                                          (dhTransform(kDH[4], t5) * dhTransform(kDH[5], t6)).inverse();
            const double px = T14.translation().x(), py = T14.translation().y();
            double c3 = (px * px + py * py - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
            if (std::abs(c3) > 1.0 + 1e-12) continue;
            c3 = std::clamp(c3, -1.0, 1.0);

            for (const double t3 : {std::acos(c3), -std::acos(c3)}) {
                const double t2 = std::atan2(py, px) -
                                  std::atan2(a3 * std::sin(t3), a2 + a3 * std::cos(t3));

                // 5. A4, rotatia ramasa
                const Eigen::Isometry3d T34 =
                    (dhTransform(kDH[1], t2) * dhTransform(kDH[2], t3)).inverse() * T14;
                const double t4 = std::atan2(T34.matrix()(1, 0), T34.matrix()(0, 0));

                Joints q;
                q << t1, t2, t3, t4, t5, t6;
                for (int i = 0; i < kAxe; ++i) q[i] = normal(q[i]);
                solutii.push_back(q);
            }
        }
    }
    return solutii;
}

std::optional<Joints> celMaiApropiat(const std::vector<Joints>& solutii, const Joints& qRef,
                                     const JointLimits& limite) {
    std::optional<Joints> best;
    double distBest = std::numeric_limits<double>::infinity();
    for (const Joints& s : solutii) {
        Joints q;
        bool valid = true;
        for (int i = 0; i < kAxe && valid; ++i) {
            double ales = 0.0, dMin = std::numeric_limits<double>::infinity();
            for (const double c : {s[i], s[i] + 2.0 * kPi, s[i] - 2.0 * kPi}) {
                if (c < limite.min[i] || c > limite.max[i]) continue;
                if (std::abs(c - qRef[i]) < dMin) {
                    dMin = std::abs(c - qRef[i]);
                    ales = c;
                }
            }
            valid = dMin < std::numeric_limits<double>::infinity();
            q[i] = ales;
        }
        if (!valid) continue;
        const double dist = (q - qRef).cwiseAbs().maxCoeff();
        if (dist < distBest) {
            distBest = dist;
            best = q;
        }
    }
    return best;
}

}  // namespace kin
