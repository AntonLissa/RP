#include "arm_sim/kinematics.hpp"
#include "arm_sim/json.hpp"
#include <fstream>
#include <stdexcept>
#include <cmath>

// Wrapping di un angolo in radianti nell’intervallo [-pi, pi]
double wrapToPi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

std::vector<std::array<double,4>> readDH(const std::string& filename)
{
    std::ifstream f(filename);
    if (!f.is_open()) {
        throw std::runtime_error("File DH non trovato: " + filename);
    }

    nlohmann::json j;
    f >> j;

    std::vector<std::array<double,4>> dh;
    for (auto& joint : j["joints"]) {
        dh.push_back({
            joint["a"].get<double>(),
            joint["alpha"].get<double>(),
            joint["d"].get<double>(),
            joint["theta"].get<double>()
        });
    }
    return dh;
}

Eigen::Matrix4d dh_transform(double a, double alpha, double d, double theta)
{
    Eigen::Matrix4d T;
    T << std::cos(theta), -std::sin(theta)*std::cos(alpha),  std::sin(theta)*std::sin(alpha), a*std::cos(theta),
         std::sin(theta),  std::cos(theta)*std::cos(alpha), -std::cos(theta)*std::sin(alpha), a*std::sin(theta),
                      0,                   std::sin(alpha),                  std::cos(alpha),               d,
                      0,                                0,                                 0,               1;
    return T;
}

Eigen::Vector3d forward_kinematics(const std::vector<std::array<double,4>>& joints)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (auto& j : joints) {
        T = T * dh_transform(j[0], j[1], j[2], j[3]);
    }
    return T.block<3,1>(0,3);
}

Eigen::MatrixXd compute_jacobian(const std::vector<std::array<double,4>>& joints)
{
    int n = joints.size();
    Eigen::MatrixXd J(3, n);
    double eps = 1e-6;

    Eigen::Vector3d f0 = forward_kinematics(joints);

    for (int i = 0; i < n; i++) {
        auto perturbed = joints;
        perturbed[i][3] += eps;
        Eigen::Vector3d f1 = forward_kinematics(perturbed);
        J.col(i) = (f1 - f0) / eps;
    }
    return J;
}

void ik_step(std::vector<std::array<double,4>>& joints, const Eigen::Vector3d& target)
{
    Eigen::Vector3d current = forward_kinematics(joints);
    Eigen::Vector3d error = target - current;

    Eigen::MatrixXd J = compute_jacobian(joints);

    Eigen::MatrixXd pinv = J.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::VectorXd delta = pinv * error * 0.1;  // gain per non scappare nell’iperspazio

    for (size_t i = 0; i < joints.size(); i++) {
        joints[i][3] += delta(i);
        joints[i][3] = wrapToPi(joints[i][3]);
        
    }
}
