#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "executor_interfaces/action/execute_task.hpp"
#include "executor_interfaces/msg/task_step.hpp"
#include "executor_interfaces/msg/task_result.hpp"
#include "hri_tools/HRI.hpp"
#include "talker_interfaces/srv/talk.hpp"
#include "known_locations_interfaces/srv/locations_get.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class SupervisorNode : public rclcpp::Node
{
public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using TaskStep    = executor_interfaces::msg::TaskStep;
    using GoalHandle  = rclcpp_action::ClientGoalHandle<ExecuteTask>;

    explicit SupervisorNode();

    // Must be called immediately after std::make_shared<SupervisorNode>().
    void init();

    ~SupervisorNode();

private:
    // ── ROS interfaces ───────────────────────────────────────────────────────
    rclcpp_action::Client<ExecuteTask>::SharedPtr executor_client_;
    std::shared_ptr<HRI> hri_;

    // Lightweight clients used only to probe service availability at startup.
    rclcpp::Client<talker_interfaces::srv::Talk>::SharedPtr          hri_check_client_;
    rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr nav_check_client_;

    // TF2 for robot pose queries (obstacle recovery heuristic).
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── Parameters ───────────────────────────────────────────────────────────
    std::string plan_name_;
    std::string plans_dir_;
    std::string hri_service_name_;
    std::string nav_service_name_;
    std::string executor_action_name_;
    std::string robot_base_frame_;

    // ── Plan ─────────────────────────────────────────────────────────────────
    std::vector<TaskStep> plan_steps_;

    // ── Navigation failure tracker ───────────────────────────────────────────
    struct NavTracker {
        std::string                   location;
        int                           failures{0};
        geometry_msgs::msg::Point     first_pose;
        bool                          has_first_pose{false};
    };
    NavTracker nav_tracker_;
    bool       obstacle_recovery_triggered_{false};

    static constexpr int    kMaxNavAttempts    = 3;
    static constexpr double kSamePoseThreshold = 0.5;   // metres

    // ── Orchestration thread ─────────────────────────────────────────────────
    std::thread       orchestration_thread_;
    std::atomic<bool> running_{true};

    // ── Orchestration stages ─────────────────────────────────────────────────
    void run();

    bool check_modules();
    bool load_plan();
    void execute_plan();

    // Execute one plan step with up to retry_limit attempts.
    bool execute_step(const TaskStep& step, int retry_limit);

    // Send one goal to the executor action server and block until result.
    bool send_and_wait(const TaskStep& step);

    // Navigation-specific failure handler.
    // Returns true when obstacle recovery is triggered (caller should abort).
    bool handle_nav_failure(const std::string& location);
    void trigger_obstacle_recovery();

    // ── Utilities ────────────────────────────────────────────────────────────
    geometry_msgs::msg::Point get_robot_pose();

    static double pose_distance(const geometry_msgs::msg::Point& a,
                                const geometry_msgs::msg::Point& b);
};
