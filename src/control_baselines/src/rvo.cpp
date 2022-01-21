#include <fstream>
#include <iostream>
#include <sstream>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/current_state.hpp"

#include <RVO.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>


using std::placeholders::_1;

struct Agent
{
    Agent(rclcpp::Node* node, const std::string& ns)
        : ns_{ns}
        , node_{node}
        , current_state_{}
    {
        cs_sub_ = node->create_subscription<freyja_msgs::msg::CurrentState>(ns + "/current_state", 1, [this](const freyja_msgs::msg::CurrentState::ConstSharedPtr msg)
        {
            current_state_ = *msg;
        });

        refstate_pub_ = node->create_publisher<freyja_msgs::msg::ReferenceState>(ns + "/reference_state", 1);
    }

    const std::string ns_;
    rclcpp::Node* node_;
    rclcpp::Subscription<freyja_msgs::msg::CurrentState>::SharedPtr cs_sub_;

    rclcpp::Publisher<freyja_msgs::msg::ReferenceState>::SharedPtr refstate_pub_;
    freyja_msgs::msg::CurrentState current_state_;
};

class RVONavigator : public rclcpp::Node
{
public:
    RVONavigator();

    rclcpp::TimerBase::SharedPtr ref_timer_;

    void sendReference();

private:

    std::vector<std::unique_ptr<Agent>> agents_;
    std::vector<RVO::Vector2> goals;
    RVO::RVOSimulator sim;
};

RVONavigator::RVONavigator()
    : Node("reference_state_node")
    , sim{}
{
    ref_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<float>(1.0 / 30.0),
                                      std::bind(&RVONavigator::sendReference, this));

    for (const auto& i :
         {
             0, 1
         })
    {
        agents_.emplace_back(std::make_unique<Agent>(this, "robomaster_" + std::to_string(i)));
    }
}


void RVONavigator::sendReference()
{
    float ve = -0.2;
    float vn = 0.0;

    for (auto& agent : agents_)
    {
        freyja_msgs::msg::ReferenceState rs{};
        rs.pn = agent->current_state_.state_vector[0];
        rs.pe = agent->current_state_.state_vector[1];
        rs.pd = agent->current_state_.state_vector[2];
        rs.vn = vn;
        rs.ve = ve;
        rs.vd = 0.0;
        agent->refstate_pub_->publish(rs);
        std::cout << agent->ns_ << " " << agent->current_state_.state_vector[0] << " " << agent->current_state_.state_vector[1] << "\n";
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVONavigator>());
    rclcpp::shutdown();
    return 0;
}
