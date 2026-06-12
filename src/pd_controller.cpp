/*
Author: Juan M. Gandarias (http://jmgandarias.com)
email: jmgandarias@uma.es

This script implements a Joint-space PD controller.
It computes the new desired accelerations 'y' to feed the inverse dynamics controller:
y = q_ddot_d + Kd*(q_dot_d - q_dot) + Kp*(q_d - q)

Since q_dot_d = 0 and q_ddot_d = 0:
y = Kp*(q_d - q) - Kd*q_dot

Inputs: joint_states
Outputs: desired_joint_accelerations
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <Eigen/Dense>

class PDControllerNode : public rclcpp::Node
{
public:
    PDControllerNode()
        : Node("pd_controller_node"),
          joint_positions_(Eigen::VectorXd::Zero(2)),
          joint_velocities_(Eigen::VectorXd::Zero(2)),
          desired_joint_accelerations_(Eigen::VectorXd::Zero(2))
    {
        // Frequency initialization
        this->declare_parameter<double>("frequency", 1000.0);

        // PD Controller parameters initialization
        this->declare_parameter<std::vector<double>>("qd", {0.785, 0.785}); // Desired position
        this->declare_parameter<double>("kp1", 100.0);
        this->declare_parameter<double>("kp2", 100.0);
        this->declare_parameter<double>("kd1", 20.0);
        this->declare_parameter<double>("kd2", 20.0);

        // Get frequency [Hz] parameter
        double frequency = this->get_parameter("frequency").as_double();

        // Get PD parameters
        qd_ = Eigen::VectorXd::Map(this->get_parameter("qd").as_double_array().data(), 2);
        
        Kp_ = Eigen::MatrixXd::Zero(2, 2);
        Kp_(0, 0) = this->get_parameter("kp1").as_double();
        Kp_(1, 1) = this->get_parameter("kp2").as_double();

        Kd_ = Eigen::MatrixXd::Zero(2, 2);
        Kd_(0, 0) = this->get_parameter("kd1").as_double();
        Kd_(1, 1) = this->get_parameter("kd2").as_double();

        // Create subscription to joint_states
        subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 1, std::bind(&PDControllerNode::joint_states_callback, this, std::placeholders::_1));

        // Create publisher for desired_joint_accelerations
        publisher_desired_accelerations_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("desired_joint_accelerations", 1);

        // Set the timer callback
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / frequency)), std::bind(&PDControllerNode::timer_callback, this));
    }

    void timer_callback()
    {
        // Calculate the stabilizing control law 'y'
        desired_joint_accelerations_ = calculate_pd_control();

        // Publish data
        publish_data();
    }

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto joint1_index = std::find(msg->name.begin(), msg->name.end(), "joint_1") - msg->name.begin();
        auto joint2_index = std::find(msg->name.begin(), msg->name.end(), "joint_2") - msg->name.begin();

        if (static_cast<std::vector<std::string>::size_type>(joint1_index) < msg->name.size() &&
            static_cast<std::vector<std::string>::size_type>(joint2_index) < msg->name.size())
        {
            joint_positions_(0) = msg->position[joint1_index];
            joint_positions_(1) = msg->position[joint2_index];
            joint_velocities_(0) = msg->velocity[joint1_index];
            joint_velocities_(1) = msg->velocity[joint2_index];
        }
    }

    Eigen::VectorXd calculate_pd_control()
    {
        // 1. Calculate position error (q_d - q)
        Eigen::VectorXd error_q = qd_ - joint_positions_;

        // 2. Since q_dot_d = 0, the velocity error is simply (0 - q_dot)
        Eigen::VectorXd error_q_dot = -joint_velocities_;

        // 3. Control law: y = Kp * error_q + Kd * error_q_dot
        // Note: q_ddot_d is 0, so it is omitted from the sum.
        Eigen::VectorXd y = Kp_ * error_q + Kd_ * error_q_dot;

        return y;
    }

    void publish_data()
    {
        auto accel_msg = std_msgs::msg::Float64MultiArray();
        accel_msg.data.assign(desired_joint_accelerations_.data(), desired_joint_accelerations_.data() + desired_joint_accelerations_.size());
        publisher_desired_accelerations_->publish(accel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_desired_accelerations_;
    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    Eigen::VectorXd desired_joint_accelerations_;
    Eigen::VectorXd qd_;
    Eigen::MatrixXd Kp_;
    Eigen::MatrixXd Kd_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}