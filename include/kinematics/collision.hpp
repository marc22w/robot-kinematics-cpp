#pragma once

#include <array>
#include <string>
#include <vector>

#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

namespace kin {

// Cilindru cu capete semisferice: toate punctele aflate la cel mult `raza` de segmentul [a, b]
struct Capsula {
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    double raza;
};

// Plan infinit (masa, perete); spatiul liber este in sensul normalei
struct Plan {
    std::string nume;
    Eigen::Vector3d punct;
    Eigen::Vector3d normala;   // vector unitar
};

struct Obstacol {
    std::string nume;
    Capsula forma;
};

struct Scena {
    std::vector<Plan> planuri;
    std::vector<Obstacol> obstacole;
};

struct RezultatColiziune {
    bool coliziune = false;
    double distanta = 0.0;        // cea mai mica distanta libera, m (negativ = se intrepatrund)
    std::string perecheCritica;   // cele doua corpuri cele mai apropiate
};

// Distanta minima dintre segmentele [p1, q1] si [p2, q2]
double distantaSegmente(const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
                        const Eigen::Vector3d& p2, const Eigen::Vector3d& q2);

// Model simplificat UR5: cate o capsula pe fiecare segment, intre originile sistemelor DH
std::array<Capsula, kAxe> capsuleUR5(const Robot& robot, const Joints& q);

// Autocoliziuni (perechi de segmente nevecine) + coliziuni cu planurile si obstacolele scenei.
// Baza este fixata pe masa, deci nu e verificata fata de mediu.
RezultatColiziune verificaColiziuni(const Robot& robot, const Joints& q, const Scena& scena = {});

// Indexul primului punct al traiectoriei in care apare o coliziune, sau -1 daca nu exista
int primaColiziune(const Robot& robot, const Trajectory& traj, const Scena& scena = {});

}  // namespace kin
