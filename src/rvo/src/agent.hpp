#include "rclcpp/rclcpp.hpp"
#include "evaluation_msgs/action/pose_control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/current_state.hpp"

#include <RVO.h>
#include <deque>

struct Agent
{
    enum State { initial, move_waypoints, move_goal, reached_goal };

    using PoseControl = evaluation_msgs::action::PoseControl;
    using GoalHandlePoseControl = rclcpp_action::ServerGoalHandle<PoseControl>;

    Agent(rclcpp::Node* node, const std::string& ns)
        : ns_{ns}
        , node_{node}
        , current_state_{nullptr}
        , pose_action_server_goal_handle{nullptr}
        , waypoints{}
        , state{State::initial}
        , prev_velocities{}
    {
        using namespace std::chrono_literals;
        using namespace std::placeholders;

        cs_sub_ = node->create_subscription<freyja_msgs::msg::CurrentState>(ns + "/current_state", 1, [this](const freyja_msgs::msg::CurrentState::ConstSharedPtr msg)
        {
            current_state_ = msg;
        });

        refstate_pub_ = node->create_publisher<freyja_msgs::msg::ReferenceState>(ns + "/reference_state", 1);

        pose_action_server_ = rclcpp_action::create_server<PoseControl>(
                                  node_,
                                  ns + "/pose_control",
                                  std::bind(&Agent::handle_goal, this, _1, _2),
                                  std::bind(&Agent::handle_cancel, this, _1),
                                  std::bind(&Agent::handle_accepted, this, _1));

        timer_avg_vel_ = rclcpp::create_timer(node_, node_->get_clock(), 500ms, std::bind(&Agent::updateAvgVelocity, this));
    }

    const std::string ns_;
    rclcpp::Node* node_;
    rclcpp::Subscription<freyja_msgs::msg::CurrentState>::SharedPtr cs_sub_;

    rclcpp::Publisher<freyja_msgs::msg::ReferenceState>::SharedPtr refstate_pub_;
    freyja_msgs::msg::CurrentState::ConstSharedPtr current_state_;

    rclcpp_action::Server<PoseControl>::SharedPtr pose_action_server_;
    std::shared_ptr<GoalHandlePoseControl> pose_action_server_goal_handle;

    std::deque<RVO::Vector2> waypoints;
    State state;

    // Keep track of previous velocities to detect if agent is stuck
    std::deque<RVO::Vector2> prev_velocities;
    rclcpp::TimerBase::SharedPtr timer_avg_vel_;
    RVO::Vector2 avg_velocity;

    const auto getPosition() const
    {
        assert(current_state_ != nullptr);
        return RVO::Vector2{static_cast<float>(current_state_->state_vector[0]), static_cast<float>(current_state_->state_vector[1])};
    }

    const auto getVelocity()const
    {
        assert(current_state_ != nullptr);
        return RVO::Vector2{static_cast<float>(current_state_->state_vector[3]), static_cast<float>(current_state_->state_vector[4])};
    }

    void updateAvgVelocity()
    {
        if (current_state_ == nullptr)
        {
            return;
        }

        prev_velocities.push_back(getVelocity());

        if (prev_velocities.size() > 5)
        {
            prev_velocities.pop_front();

            avg_velocity = RVO::Vector2{};

            for (const auto& vel : prev_velocities)
            {
                avg_velocity += vel;
            }

            avg_velocity /= prev_velocities.size();
        }
    }


    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const PoseControl::Goal> action_goal)
    {
        if (current_state_ == nullptr)
        {
            return rclcpp_action::GoalResponse::REJECT;
        }
        waypoints.clear();

        const auto goal = RVO::Vector2(action_goal->goal_pose.position.x, action_goal->goal_pose.position.y);
        if ((goal.y() < 0.f) && (getPosition().y() > 0.f))
        {
            waypoints.emplace_back(0.0, 0.3);
            waypoints.emplace_back(0.0, -0.3);
        }
        else if ((goal.y() >= 0.f) && (getPosition().y() <= 0.f))
        {
            waypoints.emplace_back(0.0, -0.3);
            waypoints.emplace_back(0.0, 0.3);
        }

        state = Agent::State::move_waypoints;
        waypoints.push_back(goal);

        RCLCPP_INFO(node_->get_logger(), "Received goal request for %s", goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePoseControl>)
    {
        switch (state)
        {
            case Agent::State::initial:
                RCLCPP_INFO(node_->get_logger(), "Cancel rejected: No goal is executed");
                return rclcpp_action::CancelResponse::REJECT;

            case Agent::State::move_waypoints:
            case Agent::State::move_goal:
            case Agent::State::reached_goal:
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