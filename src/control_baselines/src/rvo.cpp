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

    const auto getPosition() const
    {
        return RVO::Vector2{static_cast<float>(current_state_.state_vector[0]), static_cast<float>(current_state_.state_vector[1])};
    }

    const auto getVelocity()const
    {
        return RVO::Vector2{static_cast<float>(current_state_.state_vector[3]), static_cast<float>(current_state_.state_vector[4])};
    }
};

class RVONavigator : public rclcpp::Node
{
public:
    RVONavigator();

    rclcpp::TimerBase::SharedPtr ref_timer_;

    void sendReference();

    void setupScenario();
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
             0, 1, 2
         })
    {
        agents_.emplace_back(std::make_unique<Agent>(this, "robomaster_" + std::to_string(i)));
    }
    goals.emplace_back(-1.0, 2.0);
    goals.emplace_back(0.0, 2.0);
    goals.emplace_back(1.0, 2.0);
    setupScenario();
}

void RVONavigator::setupScenario()
{
    // Specify global time step of the simulation.
    sim.setTimeStep(1.0 / 30.0);

    // Specify default parameters for agents that are subsequently added.
    sim.setAgentDefaults(15.0f, 10, 5.0f, 2.0f, 0.3f, 2.0f);

    // Add agents, specifying their start position.
    for (int i = 0; i < static_cast<int>(agents_.size()); i++)
    {
        sim.addAgent(RVO::Vector2(0.0f, static_cast<float>(i)));
    }

    // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
    std::vector<RVO::Vector2> vertices;
    vertices.push_back(RVO::Vector2(-10.0f, -3.0f));
    vertices.push_back(RVO::Vector2(-0.5f, -0.2f));
    vertices.push_back(RVO::Vector2(-0.5f, 0.2f));
    vertices.push_back(RVO::Vector2(-10.0f, 3.0f));

    sim.addObstacle(vertices);

    // Process obstacles so that they are accounted for in the simulation.
    sim.processObstacles();
}

void RVONavigator::sendReference()
{
    for (size_t i = 0; i < sim.getNumAgents(); i++)
    {
        const auto p = agents_[i]->getPosition();
        sim.setAgentPosition(i, p);

        const auto dp = goals[i] - p;

        if (abs(dp) < sim.getAgentRadius(i) )
        {
            sim.setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
        }
        else
        {
            const auto v_ref = normalize(dp) * sim.getAgentMaxSpeed(i);
            sim.setAgentPrefVelocity(i, v_ref);
        }
    }

    sim.doStep();

    for (size_t i = 0; i < sim.getNumAgents(); i++)
    {
        freyja_msgs::msg::ReferenceState rs{};
        rs.pn = agents_[i]->current_state_.state_vector[0];
        rs.pe = agents_[i]->current_state_.state_vector[1];
        rs.pd = agents_[i]->current_state_.state_vector[2];
        auto v_des = sim.getAgentVelocity(i);
        const auto v_true = agents_[i]->getVelocity();

        float maxAccel = 10.0;
        float dv = abs(v_des - v_true);
        std::cout << i << ": " << dv << " " << (maxAccel * sim.getTimeStep()) << " dv: " << dv;
        std::cout << " v_des: " << v_des << " v_true: " << v_true;

        if (dv > maxAccel * sim.getTimeStep())
        {
            const auto v_des_constr = (1 - (maxAccel * sim.getTimeStep() / dv)) * v_true + (maxAccel * sim.getTimeStep() / dv) * v_des;
            v_des = v_des_constr;
            std::cout << " constr: " << v_des_constr;
        }

        std::cout <<  "\n";
        rs.vn = v_des.x();
        rs.ve = v_des.y();
        rs.vd = 0.0;
        agents_[i]->refstate_pub_->publish(rs);
        //std::cout << agents_[i]->ns_ << " " << agents_[i]->current_state_.state_vector[0] << " " << agents_[i]->current_state_.state_vector[1] << "\n";
        //std::cout << agents_[i]->ns_ << " " << v_true << "\n";
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVONavigator>());
    rclcpp::shutdown();
    return 0;
}
