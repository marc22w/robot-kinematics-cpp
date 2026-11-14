#include "kinematics/robot.hpp"

#include <algorithm>
#include <cmath>
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

// O singura incercare, pornind din q
IKResult incercare(const Robot& robot, const Eigen::Isometry3d& tinta, Joints q,
                   const IKOptions& opt) {
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

        const Joints q_nou = q + dq;
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

IKResult Robot::inverse(const Eigen::Isometry3d& tinta, const Joints& start,
                        const IKOptions& opt) const {
    std::mt19937 gen(opt.seed);
    std::uniform_real_distribution<double> u(-kPi, kPi);

    Joints q0 = start;
    IKResult best;
    int total = 0;
    for (int r = 0; r <= opt.restarts; ++r) {
        const IKResult res = incercare(*this, tinta, q0, opt);
        total += res.iterations;
        if (r == 0 || res.posError + res.rotError < best.posError + best.rotError) best = res;
        if (res.converged) break;
        for (int i = 0; i < kAxe; ++i) q0[i] = u(gen);   // pornim din alt punct
    }
    best.iterations = total;
    for (int i = 0; i < kAxe; ++i) {                     // unghiuri in [-pi, pi]
        best.q[i] = std::atan2(std::sin(best.q[i]), std::cos(best.q[i]));
    }
    return best;
}

}  // namespace kin
