#include "trajectories.hpp"
#include <cmath>

std::vector<Eigen::Vector3d> generateCircleTrajectory(double radius, const Eigen::Vector3d& center, int n_points) {
    std::vector<Eigen::Vector3d> points;
    for(int i = 0; i < n_points; i++) {
        double theta = 2*M_PI*i/n_points;
        points.push_back(center + Eigen::Vector3d(radius*cos(theta), radius*sin(theta), 0));
    }
    return points;
}

std::vector<Eigen::Vector3d> generateEllipseTrajectory(double a, double b, const Eigen::Vector3d& center, int n_points) {
    std::vector<Eigen::Vector3d> points;
    for(int i = 0; i < n_points; i++) {
        double theta = 2*M_PI*i/n_points;
        points.push_back(center + Eigen::Vector3d(a*cos(theta), b*sin(theta), 0));
    }
    return points;
}


std::vector<Eigen::Vector3d> generateHeartTrajectory(double scale, const Eigen::Vector3d& center, int n_points) {
    std::vector<Eigen::Vector3d> points;
    for(int i = 0; i < n_points; i++) {
        double t = 2 * M_PI * i / n_points;
        double x = scale * 16 * pow(sin(t), 3);
        double y = scale * (13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t));
        points.push_back(center + Eigen::Vector3d(x, y, 0));
    }
    return points;
}

std::vector<Eigen::Vector3d> interpolateTrajectory(
    const std::vector<Eigen::Vector3d>& points, int steps_per_segment)
{
    std::vector<Eigen::Vector3d> interpolated;
    for (size_t i = 0; i < points.size(); i++) {
        Eigen::Vector3d start = points[i];
        Eigen::Vector3d end   = points[(i+1) % points.size()]; // ciclo continuo
        for (int s = 0; s < steps_per_segment; s++) {
            double alpha = double(s) / steps_per_segment;
            interpolated.push_back(start + alpha * (end - start));
        }
    }
    return interpolated;
}
