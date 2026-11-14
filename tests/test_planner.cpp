#include <gtest/gtest.h>

#include "kinematics/collision.hpp"
#include "kinematics/planner.hpp"
#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

using namespace kin;

namespace {

constexpr double kGrad = 3.14159265358979323846 / 180.0;

Joints grade(double a1, double a2, double a3, double a4, double a5, double a6) {
    Joints q;
    q << a1, a2, a3, a4, a5, a6;
    return q * kGrad;
}

// robotul se roteste pe A1 de la -60 la +60 de grade; un stalp sta exact in dreptul sculei, la A1 = 0
const Joints kStart = grade(-60, -90, 100, -100, -90, 0);
const Joints kTinta = grade(60, -90, 100, -100, -90, 0);
const Joints kMijloc = grade(0, -90, 100, -100, -90, 0);

Scena scenaStalp(const Robot& r) {
    const Eigen::Vector3d p = r.forward(kMijloc).translation();
    Scena s;
    s.planuri.push_back({"masa", Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)});
    s.obstacole.push_back({"stalp", {Eigen::Vector3d(p.x(), p.y(), 0.0), Eigen::Vector3d(p.x(), p.y(), 0.55), 0.04}});
    return s;
}

}  // namespace

TEST(Planificare, DrumulDirectLovesteStalpul) {
    const Robot r = Robot::ur5();
    const Scena s = scenaStalp(r);
    EXPECT_TRUE(configuratieLibera(r, kStart, s));
    EXPECT_TRUE(configuratieLibera(r, kTinta, s));
    EXPECT_FALSE(configuratieLibera(r, kMijloc, s));
    EXPECT_FALSE(segmentLiber(r, kStart, kTinta, s, 0.05));
}

TEST(Planificare, RRTGasesteUnDrumLiber) {
    const Robot r = Robot::ur5();
    const Scena s = scenaStalp(r);
    RRTOptions opt;
    const RezultatRRT res = planificaRRT(r, kStart, kTinta, s, opt);
    ASSERT_TRUE(res.gasit) << res.motiv;
    ASSERT_GE(res.cale.size(), 3u);   // trebuie sa ocoleasca, deci cel putin un punct intermediar
    EXPECT_LT((res.cale.front() - kStart).norm(), 1e-12);
    EXPECT_LT((res.cale.back() - kTinta).norm(), 1e-12);
    for (size_t i = 1; i < res.cale.size(); ++i) {
        EXPECT_TRUE(segmentLiber(r, res.cale[i - 1], res.cale[i], s, opt.rezolutie, opt.margine));
    }
    EXPECT_LT(lungimeCale(res.cale), 4.0 * lungimeCale({kStart, kTinta}));
}

TEST(Planificare, TraiectoriaPeDrumNuLovesteNimic) {
    const Robot r = Robot::ur5();
    const Scena s = scenaStalp(r);
    const RezultatRRT res = planificaRRT(r, kStart, kTinta, s);
    ASSERT_TRUE(res.gasit);
    const Trajectory traj =
        traiectorieDinCale(res.cale, Joints::Constant(3.14159265358979323846), Joints::Constant(4.0), 0.01);
    EXPECT_LT((traj.front().q - kStart).norm(), 1e-12);
    EXPECT_LT((traj.back().q - kTinta).norm(), 1e-12);
    EXPECT_EQ(primaColiziune(r, traj, s), -1);
    for (size_t k = 1; k < traj.size(); ++k) EXPECT_GT(traj[k].t, traj[k - 1].t);
}

TEST(Planificare, AcelasiSeedAcelasiDrum) {
    const Robot r = Robot::ur5();
    const Scena s = scenaStalp(r);
    const RezultatRRT a = planificaRRT(r, kStart, kTinta, s);
    const RezultatRRT b = planificaRRT(r, kStart, kTinta, s);
    ASSERT_EQ(a.cale.size(), b.cale.size());
    for (size_t i = 0; i < a.cale.size(); ++i) EXPECT_LT((a.cale[i] - b.cale[i]).norm(), 1e-15);
}

TEST(Planificare, TintaInColiziuneEsteRefuzata) {
    const Robot r = Robot::ur5();
    const RezultatRRT res = planificaRRT(r, kStart, kMijloc, scenaStalp(r));
    EXPECT_FALSE(res.gasit);
    EXPECT_NE(res.motiv.find("tinta"), std::string::npos);
}

TEST(Planificare, FaraObstacoleDrumulEsteDirect) {
    const Robot r = Robot::ur5();
    Scena s;
    s.planuri.push_back({"masa", Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)});
    const RezultatRRT res = planificaRRT(r, kStart, kTinta, s);
    ASSERT_TRUE(res.gasit);
    EXPECT_EQ(res.cale.size(), 2u);
}
