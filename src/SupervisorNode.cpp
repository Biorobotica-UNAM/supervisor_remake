#include "supervisor_remake/SupervisorNode.hpp"

#include <tf2/exceptions.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <future>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / init / destructor
// ─────────────────────────────────────────────────────────────────────────────

SupervisorNode::SupervisorNode()
: Node("supervisor_node")
{
    declare_parameter<std::string>("plan",             "");
    declare_parameter<std::string>("plans_dir",        "");
    declare_parameter<std::string>("executor_action",  "/executor/execute_task");
    declare_parameter<std::string>("robot_base_frame", "base_link");
}

void SupervisorNode::init()
{
    plan_name_            = get_parameter("plan").as_string();
    plans_dir_            = get_parameter("plans_dir").as_string();
    executor_action_name_ = get_parameter("executor_action").as_string();
    robot_base_frame_     = get_parameter("robot_base_frame").as_string();

    auto self = shared_from_this();

    // HRI facade — used by the supervisor for startup announcements.
    hri_ = std::make_shared<HRI>(self);

    // Health-check clients (only wait_for_service is called on these).
    const rmw_qos_profile_t qos = rmw_qos_profile_services_default;
    hri_check_client_ = create_client<talker_interfaces::srv::Talk>(
        "/hri/talker/talk", qos);
    nav_check_client_ = create_client<known_locations_interfaces::srv::LocationsGet>(
        "/nav_controller/go_to_location", qos);

    // Executor action client.
    executor_client_ = rclcpp_action::create_client<ExecuteTask>(this, executor_action_name_);

    // TF2 buffer/listener for pose tracking in the obstacle recovery heuristic.
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Launch the orchestration loop in a background thread so rclcpp::spin()
    // keeps running in the main thread and can process all callbacks.
    orchestration_thread_ = std::thread(&SupervisorNode::run, this);

    RCLCPP_INFO(get_logger(), "SupervisorNode initialized. Plan: '%s'", plan_name_.c_str());
}

SupervisorNode::~SupervisorNode()
{
    running_ = false;
    if (orchestration_thread_.joinable()) {
        orchestration_thread_.join();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main orchestration loop (background thread)
// ─────────────────────────────────────────────────────────────────────────────

void SupervisorNode::run()
{
    RCLCPP_INFO(get_logger(), "Orchestration thread started.");

    if (!check_modules()) {
        RCLCPP_ERROR(get_logger(), "Module check failed. System halted.");
        return;
    }

    if (plan_name_.empty()) {
        RCLCPP_INFO(get_logger(), "No plan provided. Entering idle state.");
        return;
    }

    if (!load_plan()) {
        hri_->Say("I could not load the task plan. Please check the plan file.");
        return;
    }

    execute_plan();

    RCLCPP_INFO(get_logger(), "Idle state. Waiting for a new command.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Module health check
// ─────────────────────────────────────────────────────────────────────────────

bool SupervisorNode::check_modules()
{
    RCLCPP_INFO(get_logger(), "Waiting for modules to initialize...");

    // Allow up to 20 s for each module to come online (navigation starts 3 s
    // after the supervisor, HRI 6 s after — launch stagger).
    const bool hri_ok = hri_check_client_->wait_for_service(20s);
    const bool nav_ok = nav_check_client_->wait_for_service(20s);

    std::vector<std::string> offline;
    if (!hri_ok) offline.push_back("HRI");
    if (!nav_ok) offline.push_back("Navigation");

    if (offline.empty()) {
        RCLCPP_INFO(get_logger(), "All modules online.");
        hri_->Say("All modules are now online.");
        return true;
    }

    std::string msg = "The following modules are offline:";
    for (const auto& m : offline) { msg += " " + m + ";"; }
    RCLCPP_ERROR(get_logger(), "%s", msg.c_str());

    if (hri_ok) {
        hri_->Say(msg);
    }

    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Plan loading (YAML)
// ─────────────────────────────────────────────────────────────────────────────

bool SupervisorNode::load_plan()
{
    if (plans_dir_.empty()) {
        RCLCPP_ERROR(get_logger(), "Parameter 'plans_dir' is empty.");
        return false;
    }

    const std::string path = plans_dir_ + "/" + plan_name_ + ".yaml";
    RCLCPP_INFO(get_logger(), "Loading plan from '%s'...", path.c_str());

    try {
        const YAML::Node config = YAML::LoadFile(path);
        const auto& steps = config["steps"];

        for (std::size_t i = 0; i < steps.size(); ++i) {
            const auto& s = steps[i];

            TaskStep step;
            step.step_id = static_cast<uint32_t>(i);
            step.agent   = "takeshi";
            step.action  = s["action"].as<std::string>();

            if (s["args"]) {
                for (const auto& arg : s["args"]) {
                    step.args.push_back(arg.as<std::string>());
                }
            }

            plan_steps_.push_back(step);

            RCLCPP_INFO(get_logger(), "  [%zu] %s(%s)",
                i,
                step.action.c_str(),
                step.args.empty() ? "" : step.args[0].c_str());
        }

        RCLCPP_INFO(get_logger(), "Plan '%s' loaded — %zu steps.",
                    plan_name_.c_str(), plan_steps_.size());
        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "YAML error in '%s': %s", path.c_str(), e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load plan: %s", e.what());
    }

    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Plan execution
// ─────────────────────────────────────────────────────────────────────────────

void SupervisorNode::execute_plan()
{
    obstacle_recovery_triggered_ = false;
    RCLCPP_INFO(get_logger(), "=== Starting plan execution (%zu steps) ===",
                plan_steps_.size());

    for (const auto& step : plan_steps_) {
        if (!running_) { break; }

        RCLCPP_INFO(get_logger(), "--- Step [%u]: %s ---",
                    step.step_id, step.action.c_str());

        // Navigation gets up to kMaxNavAttempts retries (obstacle policy applies).
        // Other actions get 3 retries.
        const int retry_limit = (step.action == "goto") ? kMaxNavAttempts : 3;
        const bool ok = execute_step(step, retry_limit);

        if (!ok) {
            RCLCPP_ERROR(get_logger(), "Plan aborted at step [%u].", step.step_id);
            if (!obstacle_recovery_triggered_) {
                hri_->Say("I could not complete the task. Aborting the plan.");
            }
            return;
        }
    }

    RCLCPP_INFO(get_logger(), "=== Plan completed successfully. ===");
    hri_->Say("I have completed the plan successfully.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Step execution with retry
// ─────────────────────────────────────────────────────────────────────────────

bool SupervisorNode::execute_step(const TaskStep& step, int retry_limit)
{
    for (int attempt = 1; attempt <= retry_limit && running_; ++attempt) {
        RCLCPP_INFO(get_logger(), "Step [%u] attempt %d/%d...",
                    step.step_id, attempt, retry_limit);

        const bool ok = send_and_wait(step);

        if (ok) {
            nav_tracker_ = {};   // reset failure tracking on any success
            return true;
        }

        RCLCPP_WARN(get_logger(), "Step [%u] attempt %d/%d failed.",
                    step.step_id, attempt, retry_limit);

        // Navigation-specific failure: track consecutive failures from the same
        // pose and trigger obstacle recovery when the limit is reached.
        if (step.action == "goto" && !step.args.empty()) {
            if (handle_nav_failure(step.args[0])) {
                return false;   // obstacle recovery triggered — abort step
            }
        }
    }

    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Send one goal to the executor and block until result
// ─────────────────────────────────────────────────────────────────────────────

bool SupervisorNode::send_and_wait(const TaskStep& step)
{
    if (!executor_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(get_logger(), "Executor action server '%s' not available.",
                     executor_action_name_.c_str());
        return false;
    }

    auto goal_msg = ExecuteTask::Goal{};
    goal_msg.task = step;

    // Phase 1: wait for the executor to accept the goal.
    auto goal_future = executor_client_->async_send_goal(goal_msg);

    if (goal_future.wait_for(10s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(),
                     "Timeout waiting for goal acceptance (step [%u]).", step.step_id);
        return false;
    }

    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal rejected by executor (step [%u]).", step.step_id);
        return false;
    }

    // Phase 2: wait for the execution result.
    // Navigation can take up to 300 s (GoToDetailed timeout) — use 320 s margin.
    auto result_future = executor_client_->async_get_result(goal_handle);

    if (result_future.wait_for(320s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(),
                     "Timeout waiting for result (step [%u]).", step.step_id);
        return false;
    }

    const auto& wrapped = result_future.get();

    // rclcpp_action::ResultCode::SUCCEEDED means the action server called succeed().
    // Our executor calls abort() on tool failure, so code != SUCCEEDED implies failure.
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(get_logger(),
                    "Step [%u] action code %d — %s",
                    step.step_id,
                    static_cast<int>(wrapped.code),
                    wrapped.result->result.message.c_str());
        return false;
    }

    const bool ok =
        (wrapped.result->result.status == executor_interfaces::msg::TaskResult::SUCCESS);

    RCLCPP_INFO(get_logger(), "Step [%u] result: %s — %s",
                step.step_id,
                ok ? "SUCCESS" : "FAILURE",
                wrapped.result->result.message.c_str());

    return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// Navigation failure tracking & obstacle recovery
// ─────────────────────────────────────────────────────────────────────────────

bool SupervisorNode::handle_nav_failure(const std::string& location)
{
    // Reset counter when targeting a new location.
    if (nav_tracker_.location != location) {
        nav_tracker_          = {};
        nav_tracker_.location = location;
    }

    nav_tracker_.failures++;

    const auto current_pose = get_robot_pose();

    // Record the pose at the very first failure attempt.
    if (!nav_tracker_.has_first_pose) {
        nav_tracker_.first_pose     = current_pose;
        nav_tracker_.has_first_pose = true;
    }

    RCLCPP_WARN(get_logger(), "[NavTracker] '%s': failure %d/%d",
                location.c_str(), nav_tracker_.failures, kMaxNavAttempts);

    if (nav_tracker_.failures >= kMaxNavAttempts) {
        const double dist = pose_distance(nav_tracker_.first_pose, current_pose);

        RCLCPP_WARN(get_logger(),
                    "[NavTracker] Limit reached. Robot moved %.2f m from first attempt.",
                    dist);

        // Obstacle recovery fires when the robot hasn't moved significantly —
        // meaning something is physically blocking the path.
        if (dist < kSamePoseThreshold) {
            trigger_obstacle_recovery();
            return true;
        }

        // Robot did move but still failed — a different navigation issue.
        // Let execute_step handle the abort without the obstacle message.
    }

    return false;
}

void SupervisorNode::trigger_obstacle_recovery()
{
    obstacle_recovery_triggered_ = true;
    RCLCPP_ERROR(get_logger(), "OBSTACLE RECOVERY TRIGGERED.");
    hri_->Say(
        "I cannot reach the goal location because there seems to be an obstacle "
        "blocking me. Human, please move it.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Utilities
// ─────────────────────────────────────────────────────────────────────────────

geometry_msgs::msg::Point SupervisorNode::get_robot_pose()
{
    geometry_msgs::msg::Point p;   // defaults to {0, 0, 0}
    try {
        const auto tf = tf_buffer_->lookupTransform(
            "map", robot_base_frame_,
            rclcpp::Time(0),
            rclcpp::Duration::from_seconds(1.0));

        p.x = tf.transform.translation.x;
        p.y = tf.transform.translation.y;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_ONCE(get_logger(),
                         "TF lookup failed (%s). Pose-based obstacle detection disabled.",
                         ex.what());
    }
    return p;
}

double SupervisorNode::pose_distance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}
