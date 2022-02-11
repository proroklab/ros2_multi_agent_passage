#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/current_state.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "evaluation_msgs/action/pose_control.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <RVO.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>


struct Agent
{
    enum State { initial, move_center, move_goal, goal_reached };

    using PoseControl = evaluation_msgs::action::PoseControl;
    using GoalHandlePoseControl = rclcpp_action::ServerGoalHandle<PoseControl>;

    Agent(rclcpp::Node* node, const std::string& ns)
        : ns_{ns}
        , node_{node}
        , current_state_{nullptr}
        , pose_action_server_goal_handle{nullptr}
        , goal{}
        , state{State::initial}
    {
        cs_sub_ = node->create_subscription<freyja_msgs::msg::CurrentState>(ns + "/current_state", 1, [this](const freyja_msgs::msg::CurrentState::ConstSharedPtr msg)
        {
            current_state_ = msg;
        });

        refstate_pub_ = node->create_publisher<freyja_msgs::msg::ReferenceState>(ns + "/reference_state", 1);

        using namespace std::placeholders;

        pose_action_server_ = rclcpp_action::create_server<PoseControl>(
                                  node_,
                                  ns + "/pose_control",
                                  std::bind(&Agent::handle_goal, this, _1, _2),
                                  std::bind(&Agent::handle_cancel, this, _1),
                                  std::bind(&Agent::handle_accepted, this, _1));

    }

    const std::string ns_;
    rclcpp::Node* node_;
    rclcpp::Subscription<freyja_msgs::msg::CurrentState>::SharedPtr cs_sub_;

    rclcpp::Publisher<freyja_msgs::msg::ReferenceState>::SharedPtr refstate_pub_;
    freyja_msgs::msg::CurrentState::ConstSharedPtr current_state_;

    rclcpp_action::Server<PoseControl>::SharedPtr pose_action_server_;
    std::shared_ptr<GoalHandlePoseControl> pose_action_server_goal_handle;

    RVO::Vector2 goal;
    State state;

    const auto getPosition() const
    {
        return RVO::Vector2{static_cast<float>(current_state_->state_vector[0]), static_cast<float>(current_state_->state_vector[1])};
    }

    const auto getVelocity()const
    {
        return RVO::Vector2{static_cast<float>(current_state_->state_vector[3]), static_cast<float>(current_state_->state_vector[4])};
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const PoseControl::Goal> action_goal)
    {
        if (current_state_ == nullptr)
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        goal = RVO::Vector2(action_goal->goal_pose.position.x, action_goal->goal_pose.position.y);

        if (((goal.y() < 0.f) && (getPosition().y() >= 0.f)) || ((goal.y() > 0.f) && (getPosition().y() <= 0.f)))
        {
            state = Agent::State::move_center;
        }
        else
        {
            state = Agent::State::move_goal;
        }

        RCLCPP_INFO(node_->get_logger(), "Received goal request for %s", goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePoseControl>)
    {
        switch (state)
        {
            case Agent::State::initial:
            case Agent::State::goal_reached:
                RCLCPP_INFO(node_->get_logger(), "Cancel rejected: No goal is executed");
                return rclcpp_action::CancelResponse::REJECT;

            case Agent::State::move_center:
            case Agent::State::move_goal:
                state = Agent::State::initial;
                RCLCPP_INFO(node_->get_logger(), "Cancel accepted");
                return rclcpp_action::CancelResponse::ACCEPT;
        }

        return rclcpp_action::CancelResponse::REJECT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePoseControl> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "handle accepted");
        pose_action_server_goal_handle = goal_handle;
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
    visualization_msgs::msg::Marker toOCRAMarker(int agent_i);
private:

    std::vector<std::unique_ptr<Agent>> agents_;
    RVO::RVOSimulator sim;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    float delta_t_;
    std::vector<visualization_msgs::msg::Marker> static_markers_;
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
    declare_parameter<float>("max_accel", 4.f);
    declare_parameter<float>("waypoint_reached_dist", 0.3f);
    declare_parameter<float>("goal_reached_dist", 0.1f);

    declare_parameter<std::vector<std::string>>("uuids", {});

    get_parameter("sim_delta_t", delta_t_);

    ref_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<float>(delta_t_),
                                      std::bind(&RVONavigator::sendReference, this));

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 5);

    for (const auto& uuid : get_parameter("uuids").get_value<std::vector<std::string>>())
    {
        agents_.emplace_back(std::make_unique<Agent>(this, uuid));
    }

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

visualization_msgs::msg::Marker RVONavigator::toOCRAMarker(int agent_i)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map_ned";
    marker.header.stamp = get_clock()->now();
    marker.ns = "ocra_lines_0";
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.scale.x = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0.0;
    marker.color.a = 1.;

    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    for (size_t i = 0; i < sim.getAgentNumORCALines(agent_i); i++)
    {
        const auto line = sim.getAgentORCALine(agent_i, i);
        const auto line_start = line.point + 5 * line.direction;
        geometry_msgs::msg::Point p_start{};
        p_start.x = line_start.x();
        p_start.y = line_start.y();
        marker.points.emplace_back(p_start);

        const auto line_end = line.point - 5 * line.direction;
        geometry_msgs::msg::Point p_end{};
        p_end.x = line_end.x();
        p_end.y = line_end.y();
        marker.points.emplace_back(p_end);
    }

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
    static_markers_.push_back(toMarker(vertices_border, "obstacle_border"));
    sim.addObstacle(vertices_border);

    std::vector <RVO::Vector2> vertices_obstacle_left =
    {
        {3.0f, 0.5f},
        {0.25f, 0.1f},
        {0.25f, -0.1f},
        {3.0f, -0.5f},
    };
    static_markers_.push_back(toMarker(vertices_obstacle_left, "obstacle_left"));
    sim.addObstacle(vertices_obstacle_left);

    std::vector <RVO::Vector2> vertices_obstacle_right =
    {
        {-3.0f, -0.5f},
        {-0.25f, -0.1f},
        {-0.25f, 0.1f},
        {-3.0f, 0.5f}
    };
    static_markers_.push_back(toMarker(vertices_obstacle_right, "obstacle_right"));
    sim.addObstacle(vertices_obstacle_right);

    sim.processObstacles();
}

template <class T>
T clamp(const T val, const T val_min, const T val_max)
{
    if (val > val_max)
    {
        return val_max;
    }

    if (val < val_min)
    {
        return val_min;
    }

    return val;
}

RVO::Vector2 clamp(const RVO::Vector2& val, const float val_min, const float val_max)
{
    return RVO::Vector2(clamp(val.x(), val_min, val_max), clamp(val.y(), val_min, val_max));
}

void RVONavigator::sendReference()
{
    for (const auto& marker : static_markers_)
    {
        marker_pub_->publish(marker);
    }

    marker_pub_->publish(toOCRAMarker(0));

    for (size_t i = 0; i < sim.getNumAgents(); i++)
    {
        if (agents_[i]->current_state_ == nullptr)
        {
            continue;
        }

        const auto p = agents_[i]->getPosition();
        sim.setAgentPosition(i, p);
        const auto v = agents_[i]->getVelocity();
        sim.setAgentVelocity(i, v);

        auto result = std::make_shared<evaluation_msgs::action::PoseControl::Result>();
        auto feedback = std::make_shared<evaluation_msgs::action::PoseControl::Feedback>();

        auto v_ref = RVO::Vector2(0.0f, 0.0f);

        switch (agents_[i]->state)
        {
            case Agent::State::initial:
            case Agent::State::goal_reached:
                break;

            case Agent::State::move_center:
                {
                    const auto dp = RVO::Vector2(0.0f, 0.0f) - p;
                    v_ref = normalize(dp) * sim.getAgentMaxSpeed(i);

                    if (abs(dp) < get_parameter("waypoint_reached_dist").get_value<float>())
                    {
                        agents_[i]->state = Agent::State::move_goal;
                    }

                    agents_[i]->pose_action_server_goal_handle->publish_feedback(feedback);

                    break;
                }

            case Agent::State::move_goal:
                {
                    const auto dp = agents_[i]->goal - p;
                    v_ref = normalize(dp) * sim.getAgentMaxSpeed(i);

                    if (abs(dp) < get_parameter("goal_reached_dist").get_value<float>())
                    {
                        agents_[i]->state = Agent::State::goal_reached;
                        agents_[i]->pose_action_server_goal_handle->succeed(result);
                    }

                    agents_[i]->pose_action_server_goal_handle->publish_feedback(feedback);

                    break;
                }
        }

        sim.setAgentPrefVelocity(i, v_ref);
    }

    sim.doStep();

    for (size_t i = 0; i < sim.getNumAgents(); i++)
    {
        if (agents_[i]->current_state_ == nullptr)
        {
            continue;
        }

        freyja_msgs::msg::ReferenceState rs{};
        rs.pn = agents_[i]->current_state_->state_vector[0];
        rs.pe = agents_[i]->current_state_->state_vector[1];
        rs.pd = agents_[i]->current_state_->state_vector[2];
        auto v_des = sim.getAgentVelocity(i);
        const auto v_true = agents_[i]->getVelocity();

        const float g = 9.81; // m/s*s
        const float maxAccel = get_parameter("max_accel").get_value<float>() * g;
        const float maxVel = get_parameter("max_speed").get_value<float>();
        float dv = abs(v_des - v_true);

        if (dv > maxAccel * sim.getTimeStep())
        {
            const auto v_des_constr = (1 - (maxAccel * sim.getTimeStep() / dv)) * v_true + (maxAccel * sim.getTimeStep() / dv) * v_des;
            v_des = v_des_constr;
        }

        rs.vn = clamp(v_des.x(), -maxVel, maxVel);
        rs.ve = clamp(v_des.y(), -maxVel, maxVel);
        rs.vd = 0.0;
        agents_[i]->refstate_pub_->publish(rs);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVONavigator>());
    rclcpp::shutdown();
    return 0;
}
