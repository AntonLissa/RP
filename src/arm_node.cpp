#include "rclcpp/rclcpp.hpp"
#include "arm_sim/kinematics.hpp"
#include "arm_sim/trajectories.hpp"

#include <Eigen/Dense>
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <vector>
#include <array>
#include <string>
#include <chrono>


class ArmNode : public rclcpp::Node {
    std::string dh_file_;
    double publish_rate_;

    std::vector<std::array<double,4>> joints_;
    std::vector<Eigen::Vector3d> trajectory_points_;
    size_t current_target_idx_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
    visualization_msgs::msg::Marker trajectory_marker_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    ArmNode() : Node("arm_node") 
    {
        loadParameters();
        joints_ = readDH(dh_file_);
        setupTrajectory();
        setupPublishers();
        setupMarker();
        startTimer();
    }

private:
    void loadParameters() 
    {
        this->declare_parameter<std::string>("dh_file", 
            "/home/tony/progetti_ros/RP/ws/arm_sim/config/3R.json");
        dh_file_ = this->get_parameter("dh_file").as_string();

        this->declare_parameter<double>("publish_rate", 10.0);
        publish_rate_ = this->get_parameter("publish_rate").as_double();
    }

    // trajectory setup
    void setupTrajectory() 
    {   //trajectory_points_ = generateCircleTrajectory(0.2, Eigen::Vector3d(0, 0.0, 0), 100); 
        //trajectory_points_ = generateEllipseTrajectory(0.5, 0.2, Eigen::Vector3d(0, 0.0, 0.5), 100);
        trajectory_points_ = {
            Eigen::Vector3d( 0.5,  0.0,  1.0),
            Eigen::Vector3d( 0.0,  0.5,  0.0),
            Eigen::Vector3d(-0.5,  0.0,  1.0),
            Eigen::Vector3d( 0.0, -0.5,  0.0)
        };

        trajectory_points_ = interpolateTrajectory(trajectory_points_, 20);
        current_target_idx_ = 0;
    }

    // Publishers setup
    void setupPublishers()
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        traj_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", 10);
    }

    // Marker visualization setup
    void setupMarker()
    {
        trajectory_marker_.header.frame_id = "base_link";
        trajectory_marker_.ns = "trajectory";
        trajectory_marker_.id = 0;
        trajectory_marker_.type = visualization_msgs::msg::Marker::POINTS;
        trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
        trajectory_marker_.scale.x = 0.02;
        trajectory_marker_.scale.y = 0.02;
        trajectory_marker_.color.r = 1.0;
        trajectory_marker_.color.g = 0.0;
        trajectory_marker_.color.b = 0.0;
        trajectory_marker_.color.a = 1.0;
    }

    // Timer setup
    void startTimer()
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(1000.0 / publish_rate_)),
            std::bind(&ArmNode::update, this)
        );
    }



    // Publish joint states
    void publishJointStates()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name.clear();
        msg.position.clear();

        for(size_t i = 0; i < joints_.size(); i++) {
            //RCLCPP_INFO(this->get_logger(), "  joint%zu = %.4f rad", i+1, joints_[i][3]);
            msg.name.push_back("joint" + std::to_string(i+1));
            msg.position.push_back(joints_[i][3]);
        }

        joint_pub_->publish(msg);
    }

    void updateMarker(const Eigen::Vector3d& tcp)
    {
        geometry_msgs::msg::Point p;
        p.x = tcp.x(); p.y = tcp.y(); p.z = tcp.z();
        trajectory_marker_.points.push_back(p);
        trajectory_marker_.header.stamp = this->now();
        traj_pub_->publish(trajectory_marker_);
    }

    // Main update function
    void update() 
    {
        if(trajectory_points_.empty()) return;

        Eigen::Vector3d target = trajectory_points_[current_target_idx_];


        ik_step(joints_, target);

        Eigen::Vector3d tcp_position = forward_kinematics(joints_);

        publishJointStates();

        updateMarker(tcp_position);

        current_target_idx_ = (current_target_idx_ + 1) % trajectory_points_.size();
    }


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
}
