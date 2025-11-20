#pragma once
#include <vector>
#include <array>
#include <Eigen/Dense>

std::vector<std::array<double,4>> readDH(const std::string& filename);

Eigen::Matrix4d dh_transform(double a, double alpha, double d, double theta);

Eigen::Vector3d forward_kinematics(const std::vector<std::array<double,4>>& joints);

Eigen::MatrixXd compute_jacobian(const std::vector<std::array<double,4>>& joints);

void ik_step(std::vector<std::array<double,4>>& joints, const Eigen::Vector3d& target);
