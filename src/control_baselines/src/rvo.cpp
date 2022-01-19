#include <fstream>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/current_state.hpp"

#include <RVO.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>


using std::placeholders::_1;

class RVONavigator : public rclcpp::Node
{
public:
    RVONavigator();

    rclcpp::Subscription<freyja_msgs::msg::CurrentState>::SharedPtr cs_sub_;

    void csCallback(const freyja_msgs::msg::CurrentState::ConstSharedPtr);

    rclcpp::Publisher<freyja_msgs::msg::ReferenceState>::SharedPtr refstate_pub_;
    rclcpp::TimerBase::SharedPtr ref_timer_;

    void sendReference();

private:
    freyja_msgs::msg::CurrentState::ConstSharedPtr current_state_;

    std::vector<RVO::Vector2> goals;
    RVO::RVOSimulator sim;
};

RVONavigator::RVONavigator()
    : Node("reference_state_node")
    , current_state_{std::make_shared<freyja_msgs::msg::CurrentState>()}
    , sim{}
{
    cs_sub_ = create_subscription<freyja_msgs::msg::CurrentState>("current_state", 1, [this](const freyja_msgs::msg::CurrentState::ConstSharedPtr msg)
    {
        current_state_ = msg;
    });

    refstate_pub_ = create_publisher<freyja_msgs::msg::ReferenceState>("reference_state", 1);
    ref_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<float>(1.0 / 30.0),
                                      std::bind(&RVONavigator::sendReference, this));
}


void RVONavigator::sendReference()
{
    float ve = 1.0;
    float vn = 0.0;

    freyja_msgs::msg::ReferenceState rs{};
    rs.pn = current_state_->state_vector[0];
    rs.pe = current_state_->state_vector[1];
    rs.pd = current_state_->state_vector[2];
    rs.vn = vn;
    rs.ve = ve;
    rs.vd = 0.0;
    refstate_pub_->publish(rs);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVONavigator>());
    rclcpp::shutdown();
    return 0;
}
