#pragma once

#include <string>
#include <vector>

#include "kinematics/collision.hpp"
#include "kinematics/robot.hpp"
#include "kinematics/trajectory.hpp"

namespace kin {

// Setari pentru planificatorul RRT-Connect, in spatiul articulatiilor
struct RRTOptions {
    double pas = 0.3;          // rad: cat creste un arbore la un pas
    double rezolutie = 0.05;   // rad: pasul cu care se verifica un segment pentru coliziuni
    double margine = 0.01;     // m: distanta minima pastrata fata de obstacole
    int iterMax = 5000;        // incercari inainte de a renunta
    int scurtari = 300;        // incercari de scurtare a drumului gasit
    unsigned seed = 1;         // pentru esantionarea aleatoare (rezultat reproductibil)
};

struct RezultatRRT {
    bool gasit = false;
    std::vector<Joints> cale;  // puncte de trecere, de la start la tinta
    int iteratii = 0;
    int noduri = 0;            // cate configuratii au fost adaugate in cei doi arbori
    std::string motiv;         // de ce nu s-a gasit un drum
};

// Configuratia nu loveste nimic si pastreaza cel putin `margine` fata de orice obstacol
bool configuratieLibera(const Robot& robot, const Joints& q, const Scena& scena, double margine = 0.0);

// Segmentul drept a -> b (in spatiul articulatiilor) e liber, verificat la fiecare `rezolutie` rad
bool segmentLiber(const Robot& robot, const Joints& a, const Joints& b, const Scena& scena,
                  double rezolutie, double margine = 0.0);

// Drum fara coliziuni de la start la tinta (RRT-Connect + scurtare)
RezultatRRT planificaRRT(const Robot& robot, const Joints& start, const Joints& tinta,
                         const Scena& scena, const RRTOptions& opt = RRTOptions{});

// Lungimea unui drum: suma, pe segmente, a celei mai mari miscari de articulatie (rad)
double lungimeCale(const std::vector<Joints>& cale);

// Traiectorie prin punctele drumului: cate un segment PTP (grad 5) la durata minima intre ele
Trajectory traiectorieDinCale(const std::vector<Joints>& cale, const Joints& vmax, const Joints& amax,
                              double dt);

}  // namespace kin
