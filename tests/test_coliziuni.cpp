#include <gtest/gtest.h>

#include "kinematics/collision.hpp"
#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

using namespace kin;
using V = Eigen::Vector3d;

namespace {

constexpr double kGrad = 3.14159265358979323846 / 180.0;

Joints grade(double a1, double a2, double a3, double a4, double a5, double a6) {
    Joints q;
    q << a1, a2, a3, a4, a5, a6;
    return q * kGrad;
}

const Joints kAcasa = grade(0, -90, 90, -90, -90, 0);

Scena scenaMasa() {
    Scena s;
    s.planuri.push_back({"masa", V(0, 0, 0), V(0, 0, 1)});   // robotul e montat pe masa
    return s;
}

Scena scenaCuStalp() {
    Scena s = scenaMasa();
    s.obstacole.push_back({"stalp", {V(-0.40, -0.32, 0.0), V(-0.40, -0.32, 1.0), 0.03}});
    return s;
}

CartesianResult linDemo(const Robot& r) {
    const Eigen::Isometry3d start = r.forward(kAcasa);
    Eigen::Isometry3d capat = start;
    capat.translation() << -0.300, -0.400, 0.250;
    capat.linear() = Eigen::AngleAxisd(30.0 * kGrad, V::UnitZ()) * start.linear();
    return linearCartesian(r, kAcasa, capat, 3.0, 0.01);
}

}  // namespace

TEST(Segmente, CazuriGeometriceSimple) {
    EXPECT_NEAR(distantaSegmente(V(0, 0, 0), V(1, 0, 0), V(0, 1, 0), V(1, 1, 0)), 1.0, 1e-12);   // paralele
    EXPECT_NEAR(distantaSegmente(V(0, 0, 0), V(1, 0, 0), V(0.5, -1, 1), V(0.5, 1, 1)), 1.0, 1e-12);  // incrucisate
    EXPECT_NEAR(distantaSegmente(V(-1, 0, 0), V(1, 0, 0), V(0, -1, 0), V(0, 1, 0)), 0.0, 1e-12);  // se intersecteaza
    EXPECT_NEAR(distantaSegmente(V(0, 0, 0), V(1, 0, 0), V(2, 0, 0), V(3, 0, 0)), 1.0, 1e-12);    // coliniare
    EXPECT_NEAR(distantaSegmente(V(0, 0, 0), V(0, 0, 0), V(3, 4, 0), V(3, 4, 0)), 5.0, 1e-12);    // doua puncte
}

TEST(Capsule, LeagaArticulatiile) {
    const Robot r = Robot::ur5();
    const auto c = capsuleUR5(r, kAcasa);
    const auto f = r.frames(kAcasa);
    for (int i = 0; i < kAxe; ++i) {
        EXPECT_LT((c[i].a - f[i].translation()).norm(), 1e-12);
        EXPECT_LT((c[i].b - f[i + 1].translation()).norm(), 1e-12);
    }
    EXPECT_LT((c[5].b - r.forward(kAcasa).translation()).norm(), 1e-12);
}

TEST(Coliziuni, PozitiaDeLucruEsteLibera) {
    const RezultatColiziune res = verificaColiziuni(Robot::ur5(), kAcasa, scenaMasa());
    EXPECT_FALSE(res.coliziune);
    EXPECT_NEAR(res.distanta, 0.029159, 1e-9);   // umarul (d1 = 89.159 mm) minus raza bratului (60 mm)
    EXPECT_EQ(res.perecheCritica, "brat - masa");
}

TEST(Coliziuni, CotPliatEsteAutocoliziune) {
    const RezultatColiziune res = verificaColiziuni(Robot::ur5(), grade(0, -90, 170, -90, -90, 0));
    EXPECT_TRUE(res.coliziune);
    EXPECT_EQ(res.perecheCritica, "baza - antebrat");
    EXPECT_NEAR(res.distanta, -0.0466556, 1e-6);
}

TEST(Coliziuni, StalpulPeTraseulLIN) {
    const Robot r = Robot::ur5();
    const CartesianResult lin = linDemo(r);
    ASSERT_TRUE(lin.ok);

    EXPECT_EQ(primaColiziune(r, lin.traj, scenaMasa()), -1);   // fara stalp: traseu liber

    const int k = primaColiziune(r, lin.traj, scenaCuStalp());
    ASSERT_GT(k, 0);
    EXPECT_GT(k, 120);
    EXPECT_LT(k, 170);
    EXPECT_EQ(verificaColiziuni(r, lin.traj[k].q, scenaCuStalp()).perecheCritica,
              "incheietura 2 - stalp");
}

TEST(Coliziuni, CoborareaSpreMasa) {
    const Robot r = Robot::ur5();
    Eigen::Isometry3d jos = r.forward(kAcasa);
    jos.translation().z() = 0.030;   // flansa la 30 mm deasupra mesei
    const CartesianResult lin = linearCartesian(r, kAcasa, jos, 2.0, 0.01);
    ASSERT_TRUE(lin.ok);

    const int k = primaColiziune(r, lin.traj, scenaMasa());
    ASSERT_GT(k, 0);
    EXPECT_EQ(verificaColiziuni(r, lin.traj[k].q, scenaMasa()).perecheCritica, "flansa - masa");
    const double z = r.forward(lin.traj[k].q).translation().z();
    EXPECT_LT(z, 0.045);   // raza capsulei flansei
    EXPECT_GT(z, 0.030);
}
