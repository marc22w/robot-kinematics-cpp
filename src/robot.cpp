#include "kinematics/robot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace kin {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Un pas de tip damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e
Joints pasDLS(const Jacobian& J, const Vector6& e, double lambda) {
    const Eigen::Matrix<double, 6, 6> A =
        J * J.transpose() + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
    return J.transpose() * A.ldlt().solve(e);
}

Joints inLimite(const Joints& q, const JointLimits& lim) {
    return q.cwiseMax(lim.min).cwiseMin(lim.max);
}

// O singura incercare, pornind din q; articulatiile raman mereu in limite
IKResult incercare(const Robot& robot, const Eigen::Isometry3d& tinta, Joints q,
                   const IKOptions& opt) {
    const JointLimits& lim = robot.limits();
    q = inLimite(q, lim);
    double lambda = 0.01;
    Vector6 e = poseError(tinta, robot.forward(q));
    IKResult res;
    int k = 0;
    for (; k < opt.maxIter; ++k) {
        if (e.head<3>().norm() < opt.tol && e.tail<3>().norm() < opt.tol) {
            res.converged = true;
            break;
        }
        Joints dq = pasDLS(robot.jacobian(q), e, lambda);
        const double cel_mai_mare = dq.cwiseAbs().maxCoeff();
        if (cel_mai_mare > opt.maxStep) dq *= opt.maxStep / cel_mai_mare;

        const Joints q_nou = inLimite(q + dq, lim);
        const Vector6 e_nou = poseError(tinta, robot.forward(q_nou));
        if (e_nou.norm() < e.norm()) {   // pas bun: il pastram, amortizam mai putin
            q = q_nou;
            e = e_nou;
            lambda = std::max(lambda * 0.5, 1e-6);
        } else {                         // pas prost: il respingem, amortizam mai mult
            lambda = std::min(lambda * 4.0, 1.0);
        }
    }
    res.q = q;
    res.iterations = k;
    res.posError = e.head<3>().norm();
    res.rotError = e.tail<3>().norm();
    return res;
}

}  // namespace

Eigen::Isometry3d dhTransform(const DH& p, double theta) {
    const double ct = std::cos(theta), st = std::sin(theta);
    const double ca = std::cos(p.alpha), sa = std::sin(p.alpha);
    Eigen::Isometry3d T;
    T.matrix() << ct, -st * ca,  st * sa, p.a * ct,
                  st,  ct * ca, -ct * sa, p.a * st,
                 0.0,       sa,       ca,      p.d,
                 0.0,      0.0,      0.0,      1.0;
    return T;
}

Vector6 poseError(const Eigen::Isometry3d& tinta, const Eigen::Isometry3d& curent) {
    Vector6 e;
    e.head<3>() = tinta.translation() - curent.translation();
    const Eigen::AngleAxisd aa(tinta.rotation() * curent.rotation().transpose());
    e.tail<3>() = aa.angle() * aa.axis();
    return e;
}

SingularitatiUR singularitatiUR5(const Joints& q) {
    const double a2 = -0.425, a3 = -0.39225, d5 = 0.09465;
    SingularitatiUR s;
    s.incheietura = std::abs(std::sin(q[4]));
    s.cot = std::abs(std::sin(q[2]));
    s.umar = std::abs(a2 * std::cos(q[1]) + a3 * std::cos(q[1] + q[2]) +
                      d5 * std::sin(q[1] + q[2] + q[3]));
    return s;
}

Robot::Robot(std::array<DH, kAxe> dh) : dh_(dh) {}

Robot Robot::ur5() {
    return Robot(std::array<DH, kAxe>{{
        {0.0,      kPi / 2, 0.089159},
        {-0.425,   0.0,     0.0},
        {-0.39225, 0.0,     0.0},
        {0.0,      kPi / 2, 0.10915},
        {0.0,     -kPi / 2, 0.09465},
        {0.0,      0.0,     0.0823},
    }});
}

bool Robot::inLimits(const Joints& q, double tol) const {
    return (q.array() >= limite_.min.array() - tol).all() &&
           (q.array() <= limite_.max.array() + tol).all();
}

Eigen::Isometry3d Robot::forward(const Joints& q) const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    for (int i = 0; i < kAxe; ++i) {
        T = T * dhTransform(dh_[i], q[i] + dh_[i].offset);
    }
    return T;
}

std::array<Eigen::Isometry3d, kAxe + 1> Robot::frames(const Joints& q) const {
    std::array<Eigen::Isometry3d, kAxe + 1> f;
    f[0] = Eigen::Isometry3d::Identity();
    for (int i = 0; i < kAxe; ++i) {
        f[i + 1] = f[i] * dhTransform(dh_[i], q[i] + dh_[i].offset);
    }
    return f;
}

Jacobian Robot::jacobian(const Joints& q) const {
    const auto f = frames(q);
    const Eigen::Vector3d flansa = f[kAxe].translation();
    Jacobian J;
    for (int i = 0; i < kAxe; ++i) {
        const Eigen::Vector3d z = f[i].rotation().col(2);   // axa articulatiei i
        const Eigen::Vector3d o = f[i].translation();        // un punct pe axa
        J.col(i) << z.cross(flansa - o), z;
    }
    return J;
}

double Robot::manipulability(const Joints& q) const {
    return std::abs(jacobian(q).determinant());
}

double Robot::conditionNumber(const Joints& q) const {
    const Eigen::JacobiSVD<Jacobian> svd(jacobian(q));
    const Vector6 s = svd.singularValues();
    if (s(kAxe - 1) <= 0.0) return std::numeric_limits<double>::infinity();
    return s(0) / s(kAxe - 1);
}

IKResult Robot::inverse(const Eigen::Isometry3d& tinta, const Joints& start,
                        const IKOptions& opt) const {
    std::mt19937 gen(opt.seed);

    // repornirile aleatoare se fac in interiorul limitelor (si in [-pi, pi] cand se poate)
    Joints lo, hi;
    for (int i = 0; i < kAxe; ++i) {
        lo[i] = std::max(limite_.min[i], -kPi);
        hi[i] = std::min(limite_.max[i], kPi);
        if (lo[i] > hi[i]) {
            lo[i] = limite_.min[i];
            hi[i] = limite_.max[i];
        }
    }

    Joints q0 = start;
    IKResult best;
    int total = 0;
    for (int r = 0; r <= opt.restarts; ++r) {
        const IKResult res = incercare(*this, tinta, q0, opt);
        total += res.iterations;
        if (r == 0 || res.posError + res.rotError < best.posError + best.rotError) best = res;
        if (res.converged) break;
        for (int i = 0; i < kAxe; ++i) {
            q0[i] = std::uniform_real_distribution<double>(lo[i], hi[i])(gen);
        }
    }
    best.iterations = total;

    // unghiuri in [-pi, pi] cand limitele permit, altfel varianta echivalenta din limite
    for (int i = 0; i < kAxe; ++i) {
        const double w = std::atan2(std::sin(best.q[i]), std::cos(best.q[i]));
        for (const double c : {w, w + 2.0 * kPi, w - 2.0 * kPi}) {
            if (c >= limite_.min[i] && c <= limite_.max[i]) {
                best.q[i] = c;
                break;
            }
        }
    }
    return best;
}

}  // namespace kin
