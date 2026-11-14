#include <gtest/gtest.h>

#include <random>

#include "kinematics/robot.hpp"

using kin::IKResult;
using kin::JointLimits;
using kin::Joints;
using kin::Robot;

namespace {

constexpr double kGrad = 3.14159265358979323846 / 180.0;

Joints grade(double a1, double a2, double a3, double a4, double a5, double a6) {
    Joints q;
    q << a1, a2, a3, a4, a5, a6;
    return q * kGrad;
}

// Limite restranse: A1 in [0, 90], A2 in [-180, 0], A3 in [0, 160] grade
JointLimits limiteRestranse() {
    JointLimits l;
    l.min = grade(0, -180, 0, -360, -360, -360);
    l.max = grade(90, 0, 160, 360, 360, 360);
    return l;
}

}  // namespace

TEST(Singularitati, DeterminantulSeDescompuneInCeleTreiMasuri) {
    // det J = |a2 a3| * sin(A3) * sin(A5) * (a2 c2 + a3 c23 + d5 s234)
    const double a2a3 = 0.425 * 0.39225;
    std::mt19937 gen(17);
    std::uniform_real_distribution<double> u(-3.14159, 3.14159);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 50; ++k) {
        Joints q;
        for (int i = 0; i < kin::kAxe; ++i) q[i] = u(gen);
        const kin::SingularitatiUR s = kin::singularitatiUR5(q);
        EXPECT_NEAR(r.manipulability(q), a2a3 * s.incheietura * s.cot * s.umar, 1e-12);
    }
}

TEST(Singularitati, PozitiaDeLucruNuEsteSingulara) {
    const Robot r = Robot::ur5();
    const Joints q = grade(0, -90, 90, -90, -90, 0);
    EXPECT_NEAR(r.manipulability(q), 0.081169273, 1e-8);
    EXPECT_NEAR(r.conditionNumber(q), 8.212, 0.01);
    EXPECT_FALSE(kin::singularitatiUR5(q).aproape());
}

TEST(Singularitati, Incheietura) {
    const Robot r = Robot::ur5();
    const Joints q = grade(30, -60, 45, -45, 0, 10);   // A5 = 0: axele 4 si 6 aliniate
    const kin::SingularitatiUR s = kin::singularitatiUR5(q);
    EXPECT_LT(s.incheietura, 1e-12);
    EXPECT_TRUE(s.aproape());
    EXPECT_LT(r.manipulability(q), 1e-12);
    EXPECT_GT(r.conditionNumber(q), 1e8);
}

TEST(Singularitati, Cot) {
    const Robot r = Robot::ur5();
    const Joints q = grade(30, -45, 0, -45, -90, 10);  // A3 = 0: brat intins
    const kin::SingularitatiUR s = kin::singularitatiUR5(q);
    EXPECT_LT(s.cot, 1e-12);
    EXPECT_GT(s.incheietura, 0.5);
    EXPECT_GT(s.umar, 0.1);
    EXPECT_TRUE(s.aproape());
    EXPECT_LT(r.manipulability(q), 1e-12);
}

TEST(Limite, VerificareaLimitelor) {
    Robot r = Robot::ur5();
    EXPECT_TRUE(r.inLimits(grade(350, -350, 0, 0, 0, 0)));   // implicit +-360 grade
    r.setLimits(limiteRestranse());
    EXPECT_TRUE(r.inLimits(grade(45, -90, 90, 0, 0, 0)));
    EXPECT_FALSE(r.inLimits(grade(-10, -90, 90, 0, 0, 0)));  // A1 sub 0
    EXPECT_FALSE(r.inLimits(grade(45, -90, 170, 0, 0, 0)));  // A3 peste 160
}

TEST(Limite, InversaRamaneInLimite) {
    Robot r = Robot::ur5();
    r.setLimits(limiteRestranse());
    const JointLimits l = r.limits();
    std::mt19937 gen(23);
    for (int k = 0; k < 50; ++k) {
        Joints q_real;
        for (int i = 0; i < kin::kAxe; ++i) {
            const double lo = std::max(l.min[i], -3.14159), hi = std::min(l.max[i], 3.14159);
            q_real[i] = std::uniform_real_distribution<double>(lo, hi)(gen);
        }
        const Eigen::Isometry3d tinta = r.forward(q_real);
        const IKResult res = r.inverse(tinta, Joints::Zero());
        ASSERT_TRUE(res.converged);
        EXPECT_TRUE(r.inLimits(res.q));
        EXPECT_LT(kin::poseError(tinta, r.forward(res.q)).norm(), 1e-7);
    }
}

TEST(Limite, TintaAccesibilaDoarInAfaraLimitelor) {
    // fiecare axa poate iesi doar +-5 grade din pozitia de lucru
    Robot r = Robot::ur5();
    const Joints acasa = grade(0, -90, 90, -90, -90, 0);
    JointLimits l;
    l.min = acasa - Joints::Constant(5 * kGrad);
    l.max = acasa + Joints::Constant(5 * kGrad);
    r.setLimits(l);

    const Eigen::Isometry3d tinta = r.forward(grade(60, -90, 90, -90, -90, 0));  // A1 = 60 grade
    const IKResult res = r.inverse(tinta, acasa);
    EXPECT_FALSE(res.converged);
    EXPECT_TRUE(r.inLimits(res.q));
}
