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

// Transformarea omogena 4x4 a unei articulatii, pentru unghiul theta
Eigen::Isometry3d dhTransform(const DH& p, double theta);

// Eroarea dintre doua poze: [diferenta de pozitie; vectorul de rotatie]
Vector6 poseError(const Eigen::Isometry3d& tinta, const Eigen::Isometry3d& curent);

class Robot {
public:
    explicit Robot(std::array<DH, kAxe> dh);

    // UR5, cu parametrii DH publicati de Universal Robots
    static Robot ur5();

    // Cinematica directa: pozitia si orientarea flansei fata de baza
    Eigen::Isometry3d forward(const Joints& q) const;

    // Sistemele de coordonate ale tuturor articulatiilor (baza + 6)
    std::array<Eigen::Isometry3d, kAxe + 1> frames(const Joints& q) const;

    // Jacobianul geometric: viteza flansei [liniara; unghiulara] = J * viteza articulatiilor
    Jacobian jacobian(const Joints& q) const;

    // Cinematica inversa (Levenberg-Marquardt / damped least squares)
    IKResult inverse(const Eigen::Isometry3d& tinta, const Joints& start,
                     const IKOptions& opt = IKOptions{}) const;

private:
    std::array<DH, kAxe> dh_;
};

}  // namespace kin
