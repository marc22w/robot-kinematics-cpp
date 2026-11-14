#include <gtest/gtest.h>

#include <random>

#include "kinematics/robot.hpp"

using kin::Joints;
using kin::Robot;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectPose(const Eigen::Isometry3d& T, const Eigen::Matrix<double, 3, 4>& asteptat,
                double tol) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(T.matrix()(i, j), asteptat(i, j), tol)
                << "elementul (" << i << ", " << j << ")";
}

Joints aleator(std::mt19937& gen) {
    std::uniform_real_distribution<double> u(-kPi, kPi);
    Joints q;
    for (int i = 0; i < kin::kAxe; ++i) q[i] = u(gen);
    return q;
}

}  // namespace

TEST(DH, TranslatieSiRotatieSimpla) {
    // a = 1 m, theta = 90 grade: originea urmatorului sistem ajunge in (0, 1, 0)
    const Eigen::Isometry3d T = kin::dhTransform({1.0, 0.0, 0.0}, kPi / 2);
    EXPECT_NEAR(T.translation().x(), 0.0, 1e-12);
    EXPECT_NEAR(T.translation().y(), 1.0, 1e-12);
    EXPECT_NEAR(T.translation().z(), 0.0, 1e-12);
}

TEST(ForwardUR5, PozitiaZero) {
    // bratul intins orizontal: x = a2 + a3, y = -(d4 + d6), z = d1 - d5
    Eigen::Matrix<double, 3, 4> asteptat;
    asteptat << 1, 0,  0, -0.81725,
                0, 0, -1, -0.19145,
                0, 1,  0, -0.005491;
    expectPose(Robot::ur5().forward(Joints::Zero()), asteptat, 1e-9);
}

TEST(ForwardUR5, BratulVertical) {
    // umarul si cotul ridicate: inaltimea = d1 + |a2| + |a3| + d5
    Joints q;
    q << 0, -kPi / 2, 0, -kPi / 2, 0, 0;
    Eigen::Matrix<double, 3, 4> asteptat;
    asteptat << -1,  0,  0,  0.0,
                 0,  0, -1, -0.19145,
                 0, -1,  0,  1.001059;
    expectPose(Robot::ur5().forward(q), asteptat, 1e-9);
}

TEST(ForwardUR5, PozitieOarecare) {
    // valori de referinta calculate independent
    Joints q;
    q << 0.3, -1.2, 1.5, -0.8, 1.1, 0.4;
    Eigen::Matrix<double, 3, 4> asteptat;
    asteptat << 0.771207485,  0.171205134, -0.613129528, -0.566673154,
               -0.620670254,  0.416237707, -0.664465655, -0.328621728,
                0.141447697,  0.892992147,  0.427267569,  0.321458742;
    expectPose(Robot::ur5().forward(q), asteptat, 1e-8);
}

TEST(ForwardUR5, RotatiaEsteMereuOrtonormala) {
    std::mt19937 gen(42);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 100; ++k) {
        const Eigen::Matrix3d R = r.forward(aleator(gen)).rotation();
        EXPECT_TRUE((R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
        EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
    }
}

TEST(ForwardUR5, Axa1RotesteRobotulInJurulLuiZ) {
    std::mt19937 gen(3);
    const Robot r = Robot::ur5();
    Joints q = aleator(gen);
    const Eigen::Vector3d p0 = r.forward(q).translation();
    q[0] += 0.7;
    const Eigen::Vector3d p1 = r.forward(q).translation();
    const Eigen::Vector3d asteptat = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) * p0;
    EXPECT_LT((p1 - asteptat).norm(), 1e-9);
}

TEST(ForwardUR5, FlansaRamaneInRaza) {
    // distanta de la umar nu poate depasi suma lungimilor segmentelor
    const double raza = 0.425 + 0.39225 + 0.10915 + 0.09465 + 0.0823;
    const Eigen::Vector3d umar(0, 0, 0.089159);
    std::mt19937 gen(11);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 1000; ++k) {
        EXPECT_LE((r.forward(aleator(gen)).translation() - umar).norm(), raza + 1e-12);
    }
}

TEST(ForwardUR5, SistemeleSeTerminaInFlansa) {
    std::mt19937 gen(7);
    const Robot r = Robot::ur5();
    const Joints q = aleator(gen);
    const auto f = r.frames(q);
    EXPECT_TRUE(f.front().matrix().isApprox(Eigen::Matrix4d::Identity()));
    EXPECT_TRUE(f.back().matrix().isApprox(r.forward(q).matrix(), 1e-12));
}

TEST(Optimizare, CinematicaRapidaEgalaCuProdusulDH) {
    // referinta: produsul transformarilor DH, calculate fara valori precalculate
    const std::array<kin::DH, kin::kAxe> dh = {{
        {0.0, kPi / 2, 0.089159}, {-0.425, 0.0, 0.0}, {-0.39225, 0.0, 0.0},
        {0.0, kPi / 2, 0.10915}, {0.0, -kPi / 2, 0.09465}, {0.0, 0.0, 0.0823},
    }};
    std::mt19937 gen(99);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 200; ++k) {
        const Joints q = aleator(gen);
        Eigen::Isometry3d ref = Eigen::Isometry3d::Identity();
        for (int i = 0; i < kin::kAxe; ++i) ref = ref * kin::dhTransform(dh[i], q[i]);
        EXPECT_LT((r.forward(q).matrix() - ref.matrix()).cwiseAbs().maxCoeff(), 1e-14);
    }
}
