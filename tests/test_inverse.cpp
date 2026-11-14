#include <gtest/gtest.h>

#include <random>

#include "kinematics/robot.hpp"

using kin::IKResult;
using kin::Joints;
using kin::Robot;

namespace {

constexpr double kPi = 3.14159265358979323846;

Joints aleator(std::mt19937& gen, double limita = kPi) {
    std::uniform_real_distribution<double> u(-limita, limita);
    Joints q;
    for (int i = 0; i < kin::kAxe; ++i) q[i] = u(gen);
    return q;
}

// Rezultatul e corect daca robotul chiar ajunge in tinta (pot exista mai multe solutii)
void expectAjungeInTinta(const Robot& r, const IKResult& res, const Eigen::Isometry3d& tinta) {
    ASSERT_TRUE(res.converged);
    const kin::Vector6 e = kin::poseError(tinta, r.forward(res.q));
    EXPECT_LT(e.head<3>().norm(), 1e-7);
    EXPECT_LT(e.tail<3>().norm(), 1e-7);
}

}  // namespace

TEST(Jacobian, CorespundeDiferentelorFinite) {
    std::mt19937 gen(21);
    const Robot r = Robot::ur5();
    const double h = 1e-6;
    for (int k = 0; k < 20; ++k) {
        const Joints q = aleator(gen);
        const kin::Jacobian J = r.jacobian(q);
        for (int i = 0; i < kin::kAxe; ++i) {
            Joints qp = q, qm = q;
            qp[i] += h;
            qm[i] -= h;
            // derivata numerica: [deplasare; rotatie] / (2h)
            const kin::Vector6 numeric = kin::poseError(r.forward(qp), r.forward(qm)) / (2 * h);
            EXPECT_LT((J.col(i) - numeric).norm(), 1e-6) << "coloana " << i;
        }
    }
}

TEST(InversaUR5, RegasestePozitiaDinApropiere) {
    std::mt19937 gen(5);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 100; ++k) {
        const Joints q_real = aleator(gen);
        const Eigen::Isometry3d tinta = r.forward(q_real);
        const Joints start = q_real + aleator(gen, 0.3);
        expectAjungeInTinta(r, r.inverse(tinta, start), tinta);
    }
}

TEST(InversaUR5, DinPozitiaZeroCuReporniri) {
    std::mt19937 gen(9);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 50; ++k) {
        const Eigen::Isometry3d tinta = r.forward(aleator(gen));
        expectAjungeInTinta(r, r.inverse(tinta, Joints::Zero()), tinta);
    }
}

TEST(InversaUR5, TintaPreaDeparteNuConverge) {
    // la peste 2 m de umar, mult peste raza robotului (~1.1 m)
    Eigen::Isometry3d tinta = Eigen::Isometry3d::Identity();
    tinta.translation() << 2.0, 0.0, 0.5;
    const IKResult res = Robot::ur5().inverse(tinta, Joints::Zero());
    EXPECT_FALSE(res.converged);
    EXPECT_GT(res.posError, 0.9);
}

TEST(InversaUR5, UnghiurileSuntInIntervalulMinusPiPi) {
    std::mt19937 gen(13);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 20; ++k) {
        const IKResult res = r.inverse(r.forward(aleator(gen)), Joints::Zero());
        for (int i = 0; i < kin::kAxe; ++i) {
            EXPECT_LE(std::abs(res.q[i]), kPi + 1e-12);
        }
    }
}
