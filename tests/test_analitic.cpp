#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "kinematics/robot.hpp"
#include "kinematics/ur_analytic.hpp"

using kin::Joints;
using kin::Robot;

namespace {

constexpr double kGrad = 3.14159265358979323846 / 180.0;

Joints aleator(std::mt19937& gen, double limita = 3.14159265358979323846) {
    std::uniform_real_distribution<double> u(-limita, limita);
    Joints q;
    for (int i = 0; i < kin::kAxe; ++i) q[i] = u(gen);
    return q;
}

// cel mai mare unghi dintre doua configuratii, tinand cont ca +-360 de grade e acelasi lucru
double diferenta(const Joints& a, const Joints& b) {
    double m = 0.0;
    for (int i = 0; i < kin::kAxe; ++i) {
        const double d = a[i] - b[i];
        m = std::max(m, std::abs(std::atan2(std::sin(d), std::cos(d))));
    }
    return m;
}

double ceaMaiApropiata(const std::vector<Joints>& solutii, const Joints& q) {
    double m = 1e9;
    for (const Joints& s : solutii) m = std::min(m, diferenta(s, q));
    return m;
}

Eigen::Isometry3d tintaDemo() {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() << 0, 1, 0,
                  1, 0, 0,
                  0, 0, -1;
    t.translation() << -0.400, -0.200, 0.300;
    return t;
}

}  // namespace

TEST(AnaliticUR5, ToateSolutiileAjungInTinta) {
    std::mt19937 gen(31);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 500; ++k) {
        const Eigen::Isometry3d tinta = r.forward(aleator(gen));
        const std::vector<Joints> sol = kin::inverseUR5(tinta);
        EXPECT_LE(sol.size(), 8u);
        EXPECT_EQ(sol.size() % 2, 0u);
        for (const Joints& q : sol) {
            EXPECT_LT((r.forward(q).matrix() - tinta.matrix()).cwiseAbs().maxCoeff(), 1e-9);
        }
    }
}

TEST(AnaliticUR5, ConfiguratiaRealaEstePrintreSolutii) {
    std::mt19937 gen(37);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 500; ++k) {
        const Joints q_real = aleator(gen);
        EXPECT_LT(ceaMaiApropiata(kin::inverseUR5(r.forward(q_real)), q_real), 1e-6);
    }
}

TEST(AnaliticUR5, OptSolutiiPentruTintaDinDemo) {
    const std::vector<Joints> sol = kin::inverseUR5(tintaDemo());
    ASSERT_EQ(sol.size(), 8u);
    Joints numeric;
    numeric << 12.44, -94.15, 113.62, -109.47, -90.00, 12.44;
    EXPECT_LT(ceaMaiApropiata(sol, numeric * kGrad), 0.01 * kGrad);
}

TEST(AnaliticUR5, SolutiaNumericaEsteUnaDintreCeleAnalitice) {
    std::mt19937 gen(41);
    const Robot r = Robot::ur5();
    for (int k = 0; k < 100; ++k) {
        const Joints q_real = aleator(gen);
        const Eigen::Isometry3d tinta = r.forward(q_real);
        const kin::IKResult num = r.inverse(tinta, q_real + aleator(gen, 0.3));
        ASSERT_TRUE(num.converged);
        EXPECT_LT(ceaMaiApropiata(kin::inverseUR5(tinta), num.q), 1e-4);
    }
}

TEST(AnaliticUR5, TintaInAfaraSpatiuluiDeLucru) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() << 2.0, 0.0, 0.5;
    EXPECT_TRUE(kin::inverseUR5(t).empty());
    t.translation() << 0.0, 0.0, 0.5;   // incheietura pe axa 1
    EXPECT_TRUE(kin::inverseUR5(t).empty());
}

TEST(AnaliticUR5, SingularitateaIncheieturii) {
    const Robot r = Robot::ur5();
    Joints q;
    q << 0.3, -1.0, 1.0, -0.5, 0.0, 0.7;   // A5 = 0
    const Eigen::Isometry3d tinta = r.forward(q);
    const std::vector<Joints> sol = kin::inverseUR5(tinta);
    ASSERT_FALSE(sol.empty());
    for (const Joints& s : sol) {
        EXPECT_LT((r.forward(s).matrix() - tinta.matrix()).cwiseAbs().maxCoeff(), 1e-9);
    }
}

TEST(AnaliticUR5, AlegereaCeleiMaiApropiateSiLimitele) {
    const std::vector<Joints> sol = kin::inverseUR5(tintaDemo());
    Joints acasa;
    acasa << 0, -90, 90, -90, -90, 0;
    acasa *= kGrad;

    // din pozitia de lucru, cea mai apropiata e solutia gasita si numeric
    const auto q = kin::celMaiApropiat(sol, acasa);
    ASSERT_TRUE(q.has_value());
    Joints asteptat;
    asteptat << 12.44, -94.15, 113.62, -109.47, -90.00, 12.44;
    EXPECT_LT(diferenta(*q, asteptat * kGrad), 0.01 * kGrad);

    // cu A1 limitata la [-180, -90] grade raman doar solutiile cu A1 = -139.31
    kin::JointLimits l;
    l.min[0] = -180 * kGrad;
    l.max[0] = -90 * kGrad;
    const auto q2 = kin::celMaiApropiat(sol, acasa, l);
    ASSERT_TRUE(q2.has_value());
    EXPECT_NEAR((*q2)[0] / kGrad, -139.31, 0.01);

    // limite imposibile: nicio solutie
    l.min[0] = 30 * kGrad;
    l.max[0] = 60 * kGrad;
    EXPECT_FALSE(kin::celMaiApropiat(sol, acasa, l).has_value());
}
