#pragma once

#include <array>
#include <Eigen/Dense>

namespace kin {

constexpr int kAxe = 6;
using Joints = Eigen::Matrix<double, kAxe, 1>;      // unghiurile articulatiilor, rad
using Vector6 = Eigen::Matrix<double, 6, 1>;        // [pozitie; orientare]
using Jacobian = Eigen::Matrix<double, 6, kAxe>;

// Parametri Denavit-Hartenberg (conventia standard) pentru o articulatie
struct DH {
    double a;             // lungimea segmentului, m
    double alpha;         // rasucirea segmentului, rad
    double d;             // deplasarea pe axa z, m
    double offset = 0.0;  // adunat la unghiul articulatiei, rad
};

// Limitele de pozitie ale articulatiilor, rad (implicit +-360 grade, ca la UR5)
struct JointLimits {
    Joints min = Joints::Constant(-6.283185307179586);
    Joints max = Joints::Constant(6.283185307179586);
};

// Setari pentru cinematica inversa
struct IKOptions {
    int maxIter = 300;       // iteratii pe o incercare
    double tol = 1e-8;       // eroare acceptata: m pentru pozitie, rad pentru orientare
    double maxStep = 0.5;    // pasul maxim pe o articulatie intr-o iteratie, rad
    int restarts = 20;       // reporniri din puncte aleatoare daca nu converge
    unsigned seed = 1;       // pentru repornirile aleatoare (rezultat reproductibil)
};

struct IKResult {
    Joints q = Joints::Zero();
    bool converged = false;
    int iterations = 0;      // total, peste toate incercarile
    double posError = 0.0;   // m
    double rotError = 0.0;   // rad
};

// Cat de aproape e UR5 de fiecare tip de singularitate (0 = exact in singularitate)
struct SingularitatiUR {
    double incheietura;  // |sin A5|: 0 cand axele 4 si 6 sunt aliniate
    double cot;          // |sin A3|: 0 cand bratul e complet intins sau pliat
    double umar;         // m: distanta orizontala a incheieturii fata de axa 1
    bool aproape(double prag = 0.02) const {   // 0.02 = ~1.1 grade, respectiv 20 mm
        return incheietura < prag || cot < prag || umar < prag;
    }
};

// Transformarea omogena 4x4 a unei articulatii, pentru unghiul theta
Eigen::Isometry3d dhTransform(const DH& p, double theta);

// Eroarea dintre doua poze: [diferenta de pozitie; vectorul de rotatie]
Vector6 poseError(const Eigen::Isometry3d& tinta, const Eigen::Isometry3d& curent);

// Analiza singularitatilor, specifica geometriei UR5
SingularitatiUR singularitatiUR5(const Joints& q);

class Robot {
public:
    explicit Robot(std::array<DH, kAxe> dh);

    // UR5, cu parametrii DH publicati de Universal Robots
    static Robot ur5();

    void setLimits(const JointLimits& limite) { limite_ = limite; }
    const JointLimits& limits() const { return limite_; }
    bool inLimits(const Joints& q, double tol = 1e-12) const;

    // Cinematica directa: pozitia si orientarea flansei fata de baza
    Eigen::Isometry3d forward(const Joints& q) const;

    // Sistemele de coordonate ale tuturor articulatiilor (baza + 6)
    std::array<Eigen::Isometry3d, kAxe + 1> frames(const Joints& q) const;

    // Jacobianul geometric: viteza flansei [liniara; unghiulara] = J * viteza articulatiilor
    Jacobian jacobian(const Joints& q) const;

    // Masuri generale de apropiere de singularitate
    double manipulability(const Joints& q) const;   // |det J|, 0 in singularitate
    double conditionNumber(const Joints& q) const;  // sigma_max / sigma_min, infinit in singularitate

    // Cinematica inversa (Levenberg-Marquardt / damped least squares), in limitele articulatiilor
    IKResult inverse(const Eigen::Isometry3d& tinta, const Joints& start,
                     const IKOptions& opt = IKOptions{}) const;

private:
    std::array<DH, kAxe> dh_;
    JointLimits limite_;
};

}  // namespace kin
