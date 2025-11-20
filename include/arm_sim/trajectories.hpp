#ifndef ARM_SIM_TRAJECTORIES_HPP
#define ARM_SIM_TRAJECTORIES_HPP

#include <vector>
#include <Eigen/Dense>

std::vector<Eigen::Vector3d> generateCircleTrajectory(double radius, const Eigen::Vector3d& center, int n_points);
std::vector<Eigen::Vector3d> generateEllipseTrajectory(double a, double b, const Eigen::Vector3d& center, int n_points);
std::vector<Eigen::Vector3d> generateHeartTrajectory(double scale, const Eigen::Vector3d& center, int n_points);
std::vector<Eigen::Vector3d> interpolateTrajectory(const std::vector<Eigen::Vector3d>& points, int steps_per_segment);
#endif
