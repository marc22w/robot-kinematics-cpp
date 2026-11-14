#pragma once

#include <array>
#include <Eigen/Dense>

namespace kin {

constexpr int kAxe = 6;
using Joints = Eigen::Matrix<double, kAxe, 1>;  // unghiurile articulatiilor, in radiani

// Parametri Denavit-Hartenberg (conventia standard) pentru o articulatie
struct DH {
    double a;             // lungimea segmentului, m
    double alpha;         // rasucirea segmentului, rad
    double d;             // deplasarea pe axa z, m
    double offset = 0.0;  // adunat la unghiul articulatiei, rad
};

// Transformarea omogena 4x4 a unei articulatii, pentru unghiul theta
Eigen::Isometry3d dhTransform(const DH& p, double theta);

class Robot {
public:
    explicit Robot(std::array<DH, kAxe> dh);

    // UR5, cu parametrii DH publicati de Universal Robots
    static Robot ur5();

    // Cinematica directa: pozitia si orientarea flansei fata de baza
    Eigen::Isometry3d forward(const Joints& q) const;

    // Sistemele de coordonate ale tuturor articulatiilor (baza + 6)
    std::array<Eigen::Isometry3d, kAxe + 1> frames(const Joints& q) const;

private:
    std::array<DH, kAxe> dh_;
};

}  // namespace kin
