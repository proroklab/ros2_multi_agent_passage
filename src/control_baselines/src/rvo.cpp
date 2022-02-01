#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/current_state.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <RVO.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>


using std::placeholders::_1;

struct Agent
{
    enum State { initial, navigating, goal_reached };

    Agent(rclcpp::Node* node, const std::string& ns)
        : ns_{ns}
        , node_{node}
        , current_state_{}
        , goal{0, 0}
        , state{State::initial}
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

    RVO::Vector2 goal;
    State state;

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

    visualization_msgs::msg::Marker toMarker(const std::vector<RVO::Vector2>& vertices, const std::string& ns);
private:

    std::vector<std::unique_ptr<Agent>> agents_;
    RVO::RVOSimulator sim;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    float delta_t_;
};

RVONavigator::RVONavigator()
    : Node("reference_state_node")
    , sim{}
{
    declare_parameter<float>("sim_delta_t", 1.f / 30.f);
    declare_parameter<float>("neighbor_dist", 5.f);
    declare_parameter<int>("max_neighbors", 10);
    declare_parameter<float>("time_horizon", 2.f);
    declare_parameter<float>("time_horizon_obst", 2.f);
    declare_parameter<float>("robot_radius", 0.2f);
    declare_parameter<float>("max_speed", 2.f);

    declare_parameter<std::vector<std::string>>("uuids", {});

    get_parameter("sim_delta_t", delta_t_);

    ref_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<float>(delta_t_),
                                      std::bind(&RVONavigator::sendReference, this));

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 5);

    for (const auto& uuid : get_parameter("uuids").get_value<std::vector<std::string>>())
    {
        agents_.emplace_back(std::make_unique<Agent>(this, uuid));
    }

    /*goals.emplace_back(-2.0, 2.0);
    goals.emplace_back(2.0, 2.0);
    goals.emplace_back(-1.0, 2.0);
    goals.emplace_back(1.0, 2.0);*/
    //goals.emplace_back(1.0, 2.0);
    setupScenario();
}

visualization_msgs::msg::Marker RVONavigator::toMarker(const std::vector<RVO::Vector2>& vertices, const std::string& ns)
{
    assert(!vertices.empty());

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map_ned";
    marker.header.stamp = get_clock()->now();
    marker.ns = ns;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0.0;
    marker.color.a = 1.;

    for (const auto v : vertices)
    {
        geometry_msgs::msg::Point p{};
        p.x = v.x();
        p.y = v.y();
        marker.points.emplace_back(p);
    }

    marker.points.emplace_back(marker.points[0]);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    return marker;
}

void RVONavigator::setupScenario()
{
    // Specify global time step of the simulation.
    sim.setTimeStep(delta_t_);

    // Specify default parameters for agents that are subsequently added.
    sim.setAgentDefaults(
        get_parameter("neighbor_dist").get_value<float>(),
        get_parameter("max_neighbors").get_value<int>(),
        get_parameter("time_horizon").get_value<float>(),
        get_parameter("time_horizon_obst").get_value<float>(),
        get_parameter("robot_radius").get_value<float>(),
        get_parameter("max_speed").get_value<float>()
    );

    // Add agents, specifying their start position.
    for (int i = 0; i < static_cast<int>(agents_.size()); i++)
    {
        sim.addAgent(RVO::Vector2(0.0f, static_cast<float>(i)));
    }

    std::vector <RVO::Vector2> vertices_border =
    {
        {3.0f, -3.0f},
        {-3.0f, -3.0f},
        {-3.0f, 3.0f},
        {3.0f, 3.0f},
    };
    marker_pub_->publish(toMarker(vertices_border, "obstacle_border"));
    sim.addObstacle(vertices_border);

    std::vector <RVO::Vector2> vertices_obstacle_left =
    {
        {3.0f, 1.0f},
        {0.5f, 0.2f},
        {0.5f, -0.2f},
        {3.0f, -1.0f},
    };
    marker_pub_->publish(toMarker(vertices_obstacle_left, "obstacle_left"));
    sim.addObstacle(vertices_obstacle_left);

    std::vector <RVO::Vector2> vertices_obstacle_right =
    {
        {-3.0f, -1.0f},
        {-0.5f, -0.2f},
        {-0.5f, 0.2f},
        {-3.0f, 1.0f}
    };
    marker_pub_->publish(toMarker(vertices_obstacle_right, "obstacle_right"));
    sim.addObstacle(vertices_obstacle_right);

    sim.processObstacles();
}

void RVONavigator::sendReference()
{
    for (size_t i = 0; i < sim.getNumAgents(); i++)
    {
        const auto p = agents_[i]->getPosition();
        sim.setAgentPosition(i, p);

        switch (agents_[i]->state)
        {
            case Agent::State::initial:
            case Agent::State::goal_reached:
                sim.setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
                break;

            case Agent::State::navigating:
                const auto dp = agents_[i]->goal - p;

                const auto v_ref = normalize(dp) * sim.getAgentMaxSpeed(i);
                sim.setAgentPrefVelocity(i, v_ref);

                if (abs(dp) < sim.getAgentRadius(i) )
                {
                    agents_[i]->state = Agent::State::goal_reached;
                }

                break;
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

        float maxAccel = 100.0;
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
