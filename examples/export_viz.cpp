// Calculeaza cateva scenarii cu biblioteca si le scrie in JSON pentru pagina docs/viz/index.html
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "kinematics/collision.hpp"
#include "kinematics/planner.hpp"
#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

namespace {

constexpr double kGrad = 3.14159265358979323846 / 180.0;

struct Scenariu {
    std::string nume;
    std::string descriere;
    kin::Trajectory traj;
    kin::Scena scena;
    std::vector<kin::Joints> repere = {};   // puncte de trecere (planificator), afisate in pagina
};

kin::Joints grade(double a1, double a2, double a3, double a4, double a5, double a6) {
    kin::Joints q;
    q << a1, a2, a3, a4, a5, a6;
    return q * kGrad;
}

// pastreaza fiecare al `pas`-lea punct, plus ultimul
kin::Trajectory rar(const kin::Trajectory& t, size_t pas) {
    kin::Trajectory r;
    for (size_t k = 0; k < t.size(); k += pas) r.push_back(t[k]);
    if ((t.size() - 1) % pas != 0) r.push_back(t.back());
    return r;
}

void vec(std::ostream& o, const Eigen::Vector3d& v) {
    o << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
}

void scrieScenariu(std::ostream& o, const kin::Robot& robot, const Scenariu& s) {
    const kin::Trajectory& tr = s.traj;
    o << "{\"nume\":\"" << s.nume << "\",\"descriere\":\"" << s.descriere << "\",\"t\":[";
    for (size_t k = 0; k < tr.size(); ++k) o << (k ? "," : "") << tr[k].t;

    o << "],\"puncte\":[";
    std::vector<Eigen::Vector3d> scula;
    for (size_t k = 0; k < tr.size(); ++k) {
        const auto f = robot.frames(tr[k].q);
        o << (k ? "," : "") << "[";
        for (int i = 0; i <= kin::kAxe; ++i) {
            if (i) o << ",";
            vec(o, f[i].translation());
        }
        o << "]";
        scula.push_back(f[kin::kAxe].translation());
    }

    o << "],\"q\":[";
    for (size_t k = 0; k < tr.size(); ++k) {
        o << (k ? "," : "") << "[";
        for (int i = 0; i < kin::kAxe; ++i) o << (i ? "," : "") << tr[k].q[i] / kGrad;
        o << "]";
    }

    std::vector<kin::RezultatColiziune> col;
    for (const kin::TrajPoint& p : tr) col.push_back(kin::verificaColiziuni(robot, p.q, s.scena));
    o << "],\"dist\":[";
    for (size_t k = 0; k < col.size(); ++k) o << (k ? "," : "") << col[k].distanta;
    o << "],\"pereche\":[";
    for (size_t k = 0; k < col.size(); ++k) o << (k ? "," : "") << "\"" << col[k].perecheCritica << "\"";

    o << "],\"viteza\":[";   // viteza sculei, m/s
    double v = 0.0;
    for (size_t k = 0; k < tr.size(); ++k) {
        if (k > 0) {
            const double dt = tr[k].t - tr[k - 1].t;
            if (dt > 1e-12) v = (scula[k] - scula[k - 1]).norm() / dt;
        }
        o << (k ? "," : "") << v;
    }

    o << "],\"obstacole\":[";
    for (size_t i = 0; i < s.scena.obstacole.size(); ++i) {
        const kin::Obstacol& ob = s.scena.obstacole[i];
        o << (i ? "," : "") << "{\"nume\":\"" << ob.nume << "\",\"a\":";
        vec(o, ob.forma.a);
        o << ",\"b\":";
        vec(o, ob.forma.b);
        o << ",\"r\":" << ob.forma.raza << "}";
    }
    o << "],\"repere\":[";
    for (size_t i = 0; i < s.repere.size(); ++i) {
        if (i) o << ",";
        vec(o, robot.forward(s.repere[i]).translation());
    }
    o << "]}";
}

}  // namespace

int main(int argc, char** argv) {
    const std::filesystem::path fisier = argc > 1 ? argv[1] : "docs/viz/scenarii.json";
    const kin::Robot robot = kin::Robot::ur5();
    const kin::Joints acasa = grade(0, -90, 90, -90, -90, 0);

    kin::Scena masa;
    masa.planuri.push_back({"masa", Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)});
    kin::Scena cuStalp = masa;
    cuStalp.obstacole.push_back(
        {"stalp", {Eigen::Vector3d(-0.40, -0.32, 0.0), Eigen::Vector3d(-0.40, -0.32, 0.70), 0.03}});

    std::vector<Scenariu> scenarii;

    // 1. LIN din demo, cu un stalp pe traseu
    const Eigen::Isometry3d start = robot.forward(acasa);
    Eigen::Isometry3d capat = start;
    capat.translation() << -0.300, -0.400, 0.250;
    capat.linear() = Eigen::AngleAxisd(30.0 * kGrad, Eigen::Vector3d::UnitZ()) * start.linear();
    const kin::CartesianResult lin = kin::linearCartesian(robot, acasa, capat, 3.0, 0.02);
    scenarii.push_back({"LIN cu stâlp",
                        "Scula merge în linie dreaptă, cu o rotire de 30°. Un stâlp de Ø60 mm stă pe traseu: "
                        "încheietura îl lovește.",
                        lin.traj, cuStalp});

    // 2. acelasi punct final, atins cu o miscare PTP
    scenarii.push_back({"PTP spre același punct",
                        "Mișcare în spațiul articulațiilor spre aceeași poziție finală: scula nu mai merge pe o "
                        "dreaptă, ci descrie o curbă.",
                        kin::quinticJoint(acasa, lin.traj.back().q, 3.0, 0.02), masa});

    // 3. LIN cu viteza limitata, pe langa axa 1
    const kin::Joints qb = grade(3.38, -107.07, 116.33, -99.26, -90, 3.38);
    Eigen::Isometry3d capatB = robot.forward(qb);
    capatB.translation().x() += 0.70;
    kin::LinViteza v;
    v.vScula = 0.5;
    v.aScula = 1.5;
    const kin::CartesianResult lim = kin::linearCartesianViteza(robot, qb, capatB, v);
    scenarii.push_back({"Viteză limitată",
                        "LIN de 700 mm pe lângă axa 1, cu scula la 0,5 m/s. Lângă bază scula încetinește automat, "
                        "ca nicio articulație să nu depășească 180°/s.",
                        rar(lim.traj, 5), masa});

    // 4. coborare spre masa
    Eigen::Isometry3d jos = start;
    jos.translation().z() = 0.030;
    scenarii.push_back({"Coborâre spre masă",
                        "LIN vertical până la 30 mm de masă: flanșa (rază 45 mm) intră în coliziune cu masa.",
                        kin::linearCartesian(robot, acasa, jos, 2.0, 0.02).traj, masa});

    // 5-6. rotire pe A1 cu un stalp in drum: PTP direct vs drum planificat cu RRT-Connect
    const kin::Joints qs = grade(-60, -90, 100, -100, -90, 0);
    const kin::Joints qt = grade(60, -90, 100, -100, -90, 0);
    const Eigen::Vector3d pm = robot.forward(grade(0, -90, 100, -100, -90, 0)).translation();
    kin::Scena scenaRRT = masa;
    scenaRRT.obstacole.push_back(
        {"stalp", {Eigen::Vector3d(pm.x(), pm.y(), 0.0), Eigen::Vector3d(pm.x(), pm.y(), 0.55), 0.04}});
    const kin::Joints vmax = kin::Joints::Constant(180.0 * kGrad);
    const kin::Joints amax = kin::Joints::Constant(4.0);
    scenarii.push_back({"PTP direct prin obstacol",
                        "Rotire pe A1 de la −60° la +60°, cu un stâlp de Ø80 mm exact în drum. Mișcarea directă "
                        "îl lovește.",
                        kin::quinticJoint(qs, qt, kin::durataMinima(qs, qt, vmax, amax), 0.02), scenaRRT});
    const kin::RezultatRRT rrt = kin::planificaRRT(robot, qs, qt, scenaRRT);
    if (rrt.gasit) {
        scenarii.push_back({"Ocolire cu RRT-Connect",
                            "Aceeași mișcare, planificată automat: doi arbori de poziții sigure cresc din start și "
                            "din țintă până se întâlnesc, apoi drumul e scurtat. Romburile marchează punctele de "
                            "trecere.",
                            kin::traiectorieDinCale(rrt.cale, vmax, amax, 0.02), scenaRRT, rrt.cale});
    }

    if (fisier.has_parent_path()) std::filesystem::create_directories(fisier.parent_path());
    std::ofstream o(fisier);
    if (!o) {
        std::cerr << "Nu pot scrie " << fisier << "\n";
        return 1;
    }
    o << std::fixed << std::setprecision(4) << "{\"robot\":\"UR5\",\"raze\":[";
    const auto caps = kin::capsuleUR5(robot, acasa);
    for (int i = 0; i < kin::kAxe; ++i) o << (i ? "," : "") << caps[i].raza;
    o << "],\"scenarii\":[";
    for (size_t i = 0; i < scenarii.size(); ++i) {
        if (i) o << ",";
        scrieScenariu(o, robot, scenarii[i]);
    }
    o << "]}\n";
    std::cout << "Salvat: " << fisier.string() << " (" << scenarii.size() << " scenarii)\n";
    return 0;
}
