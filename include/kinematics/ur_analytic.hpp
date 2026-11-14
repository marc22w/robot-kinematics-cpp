#pragma once

#include <optional>
#include <vector>

#include "kinematics/robot.hpp"

namespace kin {

// Toate solutiile cinematicii inverse UR5 (pana la 8), calculate analitic.
// Vector gol daca tinta e in afara spatiului de lucru.
// In singularitatea incheieturii (A5 = 0), A6 este ales 0.
std::vector<Joints> inverseUR5(const Eigen::Isometry3d& tinta);

// Solutia cea mai apropiata de qRef (cel mai mic unghi maxim de miscat pe o axa),
// folosind pentru fiecare axa varianta +-360 de grade care e in limite.
// Gol daca nicio solutie nu respecta limitele.
std::optional<Joints> celMaiApropiat(const std::vector<Joints>& solutii, const Joints& qRef,
                                     const JointLimits& limite = JointLimits{});

}  // namespace kin
