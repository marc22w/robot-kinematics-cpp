#include "kinematics/robot.hpp"

#include <cmath>

namespace kin {

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

Robot::Robot(std::array<DH, kAxe> dh) : dh_(dh) {}

Robot Robot::ur5() {
    constexpr double pi = 3.14159265358979323846;
    return Robot(std::array<DH, kAxe>{{
        {0.0,      pi / 2, 0.089159},
        {-0.425,   0.0,    0.0},
        {-0.39225, 0.0,    0.0},
        {0.0,      pi / 2, 0.10915},
        {0.0,     -pi / 2, 0.09465},
        {0.0,      0.0,    0.0823},
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

}  // namespace kin
