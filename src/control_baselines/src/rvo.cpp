#include <fstream>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/current_state.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>


using std::placeholders::_1;

class ReferenceManager : public rclcpp::Node
{
public:
    ReferenceManager();

    rclcpp::Subscription<freyja_msgs::msg::CurrentState>::SharedPtr cs_sub_;

    void csCallback(const freyja_msgs::msg::CurrentState::ConstSharedPtr);

    rclcpp::Publisher<freyja_msgs::msg::ReferenceState>::SharedPtr refstate_pub_;
    rclcpp::TimerBase::SharedPtr ref_timer_;

    void sendReference();

private:
    freyja_msgs::msg::CurrentState::ConstSharedPtr current_state_;
};

ReferenceManager::ReferenceManager()
    : Node("reference_state_node")
    , current_state_{std::make_shared<freyja_msgs::msg::CurrentState>()}
{
    cs_sub_ = create_subscription<freyja_msgs::msg::CurrentState>("current_state", 1, [this](const freyja_msgs::msg::CurrentState::ConstSharedPtr msg)
    {
        current_state_ = msg;
    });

    refstate_pub_ = create_publisher<freyja_msgs::msg::ReferenceState>("reference_state", 1);
    ref_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<float>(1.0 / 30.0),
                                      std::bind(&ReferenceManager::sendReference, this));
}

void ReferenceManager::sendReference()
{
    freyja_msgs::msg::ReferenceState rs{};
    refstate_pub_->publish(rs);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReferenceManager>());
    rclcpp::shutdown();
    return 0;
}
