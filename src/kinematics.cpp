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
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("File DH non trovato: " + filename);
    }

    // Carico il JSON
    nlohmann::json j;
    file >> j;

    // Vettore dei parametri DH
    std::vector<std::array<double,4>> dh;


    for (auto& joint : j["joints"]) {

        std::array<double,4> params;

        params[0] = joint["a"].get<double>();
        params[1] = joint["alpha"].get<double>();
        params[2] = joint["d"].get<double>();
        params[3] = joint["theta"].get<double>();

        dh.push_back(params);
    }

    return dh;
}


Eigen::Matrix4d dh_transform(double a, double alpha, double d, double theta)
{
    Eigen::Matrix4d T; // Eigen::Matrix<double,4, 4> T;
    T << std::cos(theta), -std::sin(theta)*std::cos(alpha),  std::sin(theta)*std::sin(alpha), a*std::cos(theta),
         std::sin(theta),  std::cos(theta)*std::cos(alpha), -std::cos(theta)*std::sin(alpha), a*std::sin(theta),
                      0,                   std::sin(alpha),                  std::cos(alpha),               d,
                      0,                                0,                                 0,               1;
    return T;
}


// riceve vettore di joints e restituisce posizione TCP
Eigen::Vector3d forward_kinematics(const std::vector<std::array<double,4>>& joints)
{

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (const auto& joint : joints) {
        double a     = joint[0];
        double alpha = joint[1];
        double d     = joint[2];
        double theta = joint[3];

        Eigen::Matrix4d T_giunto = dh_transform(a, alpha, d, theta);

        T = T * T_giunto;
    }

    // Estraggo la posizione x, y, z dalla colonna finale della matrice 4x4
    Eigen::Vector3d posizione;
    posizione = T.block<3,1>(0,3); // estrae <3r, 1c>, da (riga 0 di colonna 3)

    return posizione;
}

Eigen::MatrixXd compute_jacobian(const std::vector<std::array<double,4>>& joints)
{

    int num_joints = joints.size();
    Eigen::MatrixXd J(3, num_joints);

    // incremento per differenziazione
    double epsilon = 1e-6;

    // posizione corrente
    Eigen::Vector3d current_position = forward_kinematics(joints);

    // Calcola la colonna i-esima di J
    for (int i = 0; i < num_joints; i++) {
        auto perturbed_joints = joints;
        perturbed_joints[i][3] += epsilon;
        Eigen::Vector3d perturbed_position = forward_kinematics(perturbed_joints);

        // Calcola la derivata parziale
        J.col(i) = (perturbed_position - current_position) / epsilon;
    }

    return J;
}

void ik_step(std::vector<std::array<double,4>>& joints, const Eigen::Vector3d& target)
{
    Eigen::Vector3d current = forward_kinematics(joints);
    Eigen::Vector3d error = target - current;

    Eigen::MatrixXd J = compute_jacobian(joints);

    Eigen::MatrixXd pinv = J.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::VectorXd delta = pinv * error * 0.1;  

    for (size_t i = 0; i < joints.size(); i++) {
        joints[i][3] += delta(i); // Aggiorna l'angolo theta
        joints[i][3] = wrapToPi(joints[i][3]);
        
    }
}
