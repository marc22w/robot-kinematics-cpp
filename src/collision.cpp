#include "kinematics/collision.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace kin {

namespace {

const std::array<double, kAxe> kRaze = {0.075, 0.060, 0.050, 0.045, 0.045, 0.045};   // m
const std::array<const char*, kAxe> kNume = {"baza", "brat", "antebrat",
                                             "incheietura 1", "incheietura 2", "flansa"};

// perechile care se pot lovi; segmentele vecine se ating oricum in articulatie
const std::array<std::pair<int, int>, 7> kPerechi = {{
    {0, 2}, {0, 3}, {0, 4}, {0, 5}, {1, 3}, {1, 4}, {1, 5},
}};

}  // namespace

double distantaSegmente(const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
                        const Eigen::Vector3d& p2, const Eigen::Vector3d& q2) {
    // algoritmul clasic: punctele cele mai apropiate, parametrii s si t limitati la [0, 1]
    const Eigen::Vector3d d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const double a = d1.squaredNorm(), e = d2.squaredNorm(), f = d2.dot(r);
    constexpr double eps = 1e-12;
    double s = 0.0, t = 0.0;

    if (a <= eps && e <= eps) return r.norm();   // ambele segmente sunt puncte
    if (a <= eps) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = d1.dot(r);
        if (e <= eps) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = d1.dot(d2), numitor = a * e - b * b;
            s = numitor > eps ? std::clamp((b * f - c * e) / numitor, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return ((p1 + d1 * s) - (p2 + d2 * t)).norm();
}

std::array<Capsula, kAxe> capsuleUR5(const Robot& robot, const Joints& q) {
    const auto f = robot.frames(q);
    std::array<Capsula, kAxe> c;
    for (int i = 0; i < kAxe; ++i) {
        c[i] = Capsula{f[i].translation(), f[i + 1].translation(), kRaze[i]};
    }
    return c;
}

RezultatColiziune verificaColiziuni(const Robot& robot, const Joints& q, const Scena& scena) {
    const auto c = capsuleUR5(robot, q);
    RezultatColiziune res;
    res.distanta = std::numeric_limits<double>::infinity();
    auto actualizeaza = [&](double d, const char* corp1, const std::string& corp2) {
        if (d < res.distanta) {
            res.distanta = d;
            res.perecheCritica = std::string(corp1) + " - " + corp2;
        }
    };

    for (const auto& [i, j] : kPerechi) {
        actualizeaza(distantaSegmente(c[i].a, c[i].b, c[j].a, c[j].b) - c[i].raza - c[j].raza,
                     kNume[i], kNume[j]);
    }
    for (int i = 1; i < kAxe; ++i) {   // baza e fixata pe masa
        for (const Plan& p : scena.planuri) {
            const double d = std::min((c[i].a - p.punct).dot(p.normala),
                                      (c[i].b - p.punct).dot(p.normala)) - c[i].raza;
            actualizeaza(d, kNume[i], p.nume);
        }
        for (const Obstacol& o : scena.obstacole) {
            actualizeaza(distantaSegmente(c[i].a, c[i].b, o.forma.a, o.forma.b) - c[i].raza -
                             o.forma.raza,
                         kNume[i], o.nume);
        }
    }
    res.coliziune = res.distanta < 0.0;
    return res;
}

int primaColiziune(const Robot& robot, const Trajectory& traj, const Scena& scena) {
    for (size_t k = 0; k < traj.size(); ++k) {
        if (verificaColiziuni(robot, traj[k].q, scena).coliziune) return static_cast<int>(k);
    }
    return -1;
}

}  // namespace kin
