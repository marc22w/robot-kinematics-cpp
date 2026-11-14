#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>

#include "kinematics/trajectory.hpp"

using namespace kin;

namespace {

constexpr double kPi = 3.14159265358979323846;

Joints dinGrade(double a1, double a2, double a3, double a4, double a5, double a6) {
    Joints q;
    q << a1, a2, a3, a4, a5, a6;
    return q * kPi / 180.0;
}

const Joints kStart = dinGrade(0, -90, 90, -90, -90, 0);

Eigen::Isometry3d tintaLiniara(const Robot& r) {
    const Eigen::Isometry3d start = r.forward(kStart);
    Eigen::Isometry3d t = start;
    t.translation() << -0.300, -0.400, 0.250;
    t.linear() = Eigen::AngleAxisd(30.0 * kPi / 180.0, Eigen::Vector3d::UnitZ()) * start.linear();
    return t;
}

}  // namespace

TEST(Profil, CapeteSiSimetrie) {
    EXPECT_DOUBLE_EQ(profil(0.0), 0.0);
    EXPECT_DOUBLE_EQ(profil(1.0), 1.0);
    EXPECT_DOUBLE_EQ(profil(0.5), 0.5);
    EXPECT_DOUBLE_EQ(profilViteza(0.0), 0.0);
    EXPECT_DOUBLE_EQ(profilViteza(1.0), 0.0);
    EXPECT_DOUBLE_EQ(profilAcceleratie(0.0), 0.0);
    EXPECT_DOUBLE_EQ(profilAcceleratie(1.0), 0.0);
    EXPECT_NEAR(profilViteza(0.5), 1.875, 1e-12);
}

TEST(Quintic, PorneseSiSeOpresteLin) {
    const Joints q1 = dinGrade(90, -45, 30, -120, -90, 45);
    const Trajectory traj = quinticJoint(kStart, q1, 2.0, 0.01);
    ASSERT_EQ(traj.size(), 201u);
    EXPECT_NEAR(traj.back().t, 2.0, 1e-12);
    EXPECT_LT((traj.front().q - kStart).norm(), 1e-12);
    EXPECT_LT((traj.back().q - q1).norm(), 1e-12);
    EXPECT_LT(traj.front().qd.norm(), 1e-12);
    EXPECT_LT(traj.back().qd.norm(), 1e-12);
    EXPECT_LT(traj.front().qdd.norm(), 1e-12);
    EXPECT_LT(traj.back().qdd.norm(), 1e-12);
}

TEST(Quintic, VitezaSiAcceleratiaSuntDerivateleCorecte) {
    const Joints q1 = dinGrade(90, -45, 30, -120, -90, 45);
    const double h = 0.001;
    const Trajectory traj = quinticJoint(kStart, q1, 2.0, h);
    for (size_t k = 1; k + 1 < traj.size(); ++k) {
        const Joints vNum = (traj[k + 1].q - traj[k - 1].q) / (2 * h);
        const Joints aNum = (traj[k + 1].qd - traj[k - 1].qd) / (2 * h);
        EXPECT_LT((vNum - traj[k].qd).norm(), 1e-4);
        EXPECT_LT((aNum - traj[k].qdd).norm(), 1e-3);
    }
}

TEST(Quintic, DurataMinimaRespectaLimitele) {
    const Joints q1 = dinGrade(120, -45, 30, -120, -90, 45);
    const Joints vmax = Joints::Constant(kPi);   // 180 grade/s
    const Joints amax = Joints::Constant(4.0);   // rad/s^2
    const double T = durataMinima(kStart, q1, vmax, amax);
    const Trajectory traj = quinticJoint(kStart, q1, T, T / 2000);

    double raportV = 0.0, raportA = 0.0;
    for (const TrajPoint& p : traj) {
        raportV = std::max(raportV, (p.qd.cwiseAbs().array() / vmax.array()).maxCoeff());
        raportA = std::max(raportA, (p.qdd.cwiseAbs().array() / amax.array()).maxCoeff());
    }
    EXPECT_LE(raportV, 1.0 + 1e-9);
    EXPECT_LE(raportA, 1.0 + 1e-9);
    EXPECT_GE(std::max(raportV, raportA), 0.999);   // o limita e atinsa: durata chiar e minima
}

TEST(Cartezian, SculaMergePeLinieDreapta) {
    const Robot r = Robot::ur5();
    const Eigen::Isometry3d start = r.forward(kStart);
    const Eigen::Isometry3d tinta = tintaLiniara(r);
    const CartesianResult res = linearCartesian(r, kStart, tinta, 3.0, 0.01);
    ASSERT_TRUE(res.ok);

    const Eigen::Vector3d a = start.translation();
    const Eigen::Vector3d ab = tinta.translation() - a;
    const Eigen::Vector3d dir = ab.normalized();
    for (const TrajPoint& p : res.traj) {
        const Eigen::Vector3d ap = r.forward(p.q).translation() - a;
        const double proiectie = ap.dot(dir);
        EXPECT_LT((ap - proiectie * dir).norm(), 1e-6);   // distanta fata de dreapta
        EXPECT_GE(proiectie, -1e-6);
        EXPECT_LE(proiectie, ab.norm() + 1e-6);
    }
    EXPECT_LT(poseError(tinta, r.forward(res.traj.back().q)).norm(), 1e-7);
}

TEST(Cartezian, ArticulatiileSeMiscaContinuu) {
    const Robot r = Robot::ur5();
    const CartesianResult res = linearCartesian(r, kStart, tintaLiniara(r), 3.0, 0.01);
    ASSERT_TRUE(res.ok);
    for (size_t k = 1; k < res.traj.size(); ++k) {
        EXPECT_LT((res.traj[k].q - res.traj[k - 1].q).cwiseAbs().maxCoeff(), 0.05);
    }
}

TEST(Cartezian, TintaInaccesibilaEsteRaportata) {
    const Robot r = Robot::ur5();
    Eigen::Isometry3d tinta = r.forward(kStart);
    tinta.translation() << 2.0, 0.0, 0.5;
    const CartesianResult res = linearCartesian(r, kStart, tinta, 3.0, 0.01);
    EXPECT_FALSE(res.ok);
    EXPECT_GT(res.esuatLa, 0);
}

TEST(CSV, UnRandPentruFiecarePunct) {
    const Trajectory traj = quinticJoint(kStart, dinGrade(10, 0, 0, 0, 0, 0), 1.0, 0.1);
    const std::string fisier = testing::TempDir() + "traiectorie_test.csv";
    scrieCSV(fisier, traj);

    std::ifstream in(fisier);
    std::string antet, rand;
    std::getline(in, antet);
    EXPECT_EQ(antet.rfind("t,q1,q2", 0), 0u);
    size_t randuri = 0;
    while (std::getline(in, rand)) ++randuri;
    EXPECT_EQ(randuri, traj.size());
}
