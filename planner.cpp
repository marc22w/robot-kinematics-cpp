#include "kinematics/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace kin {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Arbore {
    std::vector<Joints> q;
    std::vector<int> parinte;
};

enum class Pas { Blocat, Avans, Ajuns };

int celMaiApropiatNod(const Arbore& a, const Joints& q) {
    int best = 0;
    double dBest = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < a.q.size(); ++i) {
        const double d = (a.q[i] - q).squaredNorm();
        if (d < dBest) {
            dBest = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

// Creste arborele spre q cu cel mult opt.pas, daca segmentul e liber
Pas extinde(Arbore& a, const Joints& q, const Robot& robot, const Scena& scena, const RRTOptions& opt) {
    const int i = celMaiApropiatNod(a, q);
    const Joints dq = q - a.q[i];
    const double L = dq.norm();
    const Joints qNou = L <= opt.pas ? q : Joints(a.q[i] + dq * (opt.pas / L));
    if (!segmentLiber(robot, a.q[i], qNou, scena, opt.rezolutie, opt.margine)) return Pas::Blocat;
    a.q.push_back(qNou);
    a.parinte.push_back(i);
    return L <= opt.pas ? Pas::Ajuns : Pas::Avans;
}

// Drumul de la nodul i inapoi pana la radacina arborelui
std::vector<Joints> drumSpreRadacina(const Arbore& a, int i) {
    std::vector<Joints> c;
    while (i != -1) {
        c.push_back(a.q[i]);
        i = a.parinte[i];
    }
    return c;
}

}  // namespace

bool configuratieLibera(const Robot& robot, const Joints& q, const Scena& scena, double margine) {
    return verificaColiziuni(robot, q, scena).distanta > margine;
}

bool segmentLiber(const Robot& robot, const Joints& a, const Joints& b, const Scena& scena,
                  double rezolutie, double margine) {
    const int n = std::max(1, static_cast<int>(std::ceil((b - a).cwiseAbs().maxCoeff() / rezolutie)));
    for (int k = 1; k <= n; ++k) {
        const Joints q = a + (b - a) * (static_cast<double>(k) / n);
        if (!configuratieLibera(robot, q, scena, margine)) return false;
    }
    return true;
}

RezultatRRT planificaRRT(const Robot& robot, const Joints& start, const Joints& tinta,
                         const Scena& scena, const RRTOptions& opt) {
    RezultatRRT res;
    if (!configuratieLibera(robot, start, scena, opt.margine)) {
        res.motiv = "pozitia de start este in coliziune";
        return res;
    }
    if (!configuratieLibera(robot, tinta, scena, opt.margine)) {
        res.motiv = "tinta este in coliziune";
        return res;
    }
    if (segmentLiber(robot, start, tinta, scena, opt.rezolutie, opt.margine)) {
        res.gasit = true;
        res.cale = {start, tinta};
        res.noduri = 2;
        return res;
    }

    // esantionare in limitele articulatiilor (si in [-pi, pi] cand se poate)
    const JointLimits& lim = robot.limits();
    Joints lo, hi;
    for (int i = 0; i < kAxe; ++i) {
        lo[i] = std::max(lim.min[i], -kPi);
        hi[i] = std::min(lim.max[i], kPi);
        if (lo[i] > hi[i]) {
            lo[i] = lim.min[i];
            hi[i] = lim.max[i];
        }
    }
    std::mt19937 gen(opt.seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    Arbore A{{start}, {-1}}, B{{tinta}, {-1}};
    bool aEsteStart = true;
    for (int it = 0; it < opt.iterMax; ++it) {
        Joints qr;
        if (u01(gen) < 0.1) {
            qr = B.q[0];   // din cand in cand tragem direct spre radacina celuilalt arbore
        } else {
            for (int i = 0; i < kAxe; ++i) qr[i] = lo[i] + (hi[i] - lo[i]) * u01(gen);
        }

        if (extinde(A, qr, robot, scena, opt) != Pas::Blocat) {
            const Joints qNou = A.q.back();
            Pas r;
            do {
                r = extinde(B, qNou, robot, scena, opt);   // celalalt arbore incearca sa ajunga la qNou
            } while (r == Pas::Avans);

            if (r == Pas::Ajuns) {
                std::vector<Joints> ca = drumSpreRadacina(A, static_cast<int>(A.q.size()) - 1);
                std::reverse(ca.begin(), ca.end());
                const std::vector<Joints> cb = drumSpreRadacina(B, static_cast<int>(B.q.size()) - 1);
                ca.insert(ca.end(), cb.begin() + 1, cb.end());   // cb[0] este chiar qNou
                if (!aEsteStart) std::reverse(ca.begin(), ca.end());

                // scurtare: daca doua puncte se pot lega direct, eliminam ce e intre ele
                for (int s = 0; s < opt.scurtari && ca.size() > 2; ++s) {
                    std::uniform_int_distribution<size_t> ui(0, ca.size() - 1);
                    size_t i = ui(gen), j = ui(gen);
                    if (i > j) std::swap(i, j);
                    if (j - i < 2) continue;
                    if (segmentLiber(robot, ca[i], ca[j], scena, opt.rezolutie, opt.margine)) {
                        ca.erase(ca.begin() + static_cast<long>(i) + 1, ca.begin() + static_cast<long>(j));
                    }
                }

                res.gasit = true;
                res.cale = std::move(ca);
                res.iteratii = it + 1;
                res.noduri = static_cast<int>(A.q.size() + B.q.size());
                return res;
            }
        }
        std::swap(A, B);
        aEsteStart = !aEsteStart;
    }
    res.iteratii = opt.iterMax;
    res.noduri = static_cast<int>(A.q.size() + B.q.size());
    res.motiv = "nu am gasit un drum liber";
    return res;
}

double lungimeCale(const std::vector<Joints>& cale) {
    double L = 0.0;
    for (size_t i = 1; i < cale.size(); ++i) L += (cale[i] - cale[i - 1]).cwiseAbs().maxCoeff();
    return L;
}

Trajectory traiectorieDinCale(const std::vector<Joints>& cale, const Joints& vmax, const Joints& amax,
                              double dt) {
    Trajectory traj;
    if (cale.empty()) return traj;
    traj.push_back(TrajPoint{0.0, cale.front()});
    for (size_t i = 1; i < cale.size(); ++i) {
        const double T = std::max(durataMinima(cale[i - 1], cale[i], vmax, amax), dt);
        const double t0 = traj.back().t;
        const Trajectory seg = quinticJoint(cale[i - 1], cale[i], T, dt);
        for (size_t k = 1; k < seg.size(); ++k) {
            TrajPoint p = seg[k];
            p.t += t0;
            traj.push_back(p);
        }
    }
    return traj;
}

}  // namespace kin
