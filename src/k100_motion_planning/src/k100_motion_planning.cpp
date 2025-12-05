#include "k100_motion_planning/k100_motion_planning.hpp"
#include "k100_motion_planning/srv/plan_pose_goal.hpp"
#include <fstream>
#include <regex>
#include <iostream>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace k100_motion_planning {

K100MotionPlanningNode::K100MotionPlanningNode() : Node("k100_motion_planning_node") {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

bool K100MotionPlanningNode::initialize() {
    RCLCPP_INFO(this->get_logger(), "========== 初始化 K100 Motion Planning Node ==========");

    // 初始化规划器
    left_planner_ = std::make_shared<MotionPlanning>(shared_from_this(), "left_arm");
    right_planner_ = std::make_shared<MotionPlanning>(shared_from_this(), "right_arm");
    both_arms_planner_ = std::make_shared<MotionPlanning>(shared_from_this(), "both_arms");

    if (!left_planner_->initialize()) {
        RCLCPP_FATAL(this->get_logger(), "左臂规划器初始化失败");
        return false;
    }
    if (!right_planner_->initialize()) {
        RCLCPP_FATAL(this->get_logger(), "右臂规划器初始化失败");
        return false;
    }
    if (!both_arms_planner_->initialize()) {
        RCLCPP_FATAL(this->get_logger(), "双臂规划器初始化失败");
        return false;
    }

    // 设置速度/加速度缩放
    left_planner_->moveGroup()->setMaxVelocityScalingFactor(0.5);
    left_planner_->moveGroup()->setMaxAccelerationScalingFactor(0.5);
    right_planner_->moveGroup()->setMaxVelocityScalingFactor(0.5);
    right_planner_->moveGroup()->setMaxAccelerationScalingFactor(0.5);
    both_arms_planner_->moveGroup()->setMaxVelocityScalingFactor(0.5);
    both_arms_planner_->moveGroup()->setMaxAccelerationScalingFactor(0.5);

    // 初始化控制器切换客户端
    switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller", rmw_qos_profile_services_default, client_callback_group_);

    // 等待 joint_states 数据就绪
    // waitForJointStates();

    // 创建服务
    plan_joint_goal_srv_ = this->create_service<k100_motion_planning::srv::PlanJointGoal>(
        "plan_joint_goal",
        std::bind(&K100MotionPlanningNode::planJointGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_
    );

    play_rosbag_srv_ = this->create_service<k100_motion_planning::srv::PlayRosbag>(
        "play_rosbag",
        std::bind(&K100MotionPlanningNode::playRosbagCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_
    );

    compute_ik_srv_ = this->create_service<k100_motion_planning::srv::ComputeIK>(
        "compute_ik",
        std::bind(&K100MotionPlanningNode::computeIKCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_
    );

    plan_pose_goal_srv_ = this->create_service<k100_motion_planning::srv::PlanPoseGoal>(
        "plan_pose_goal",
        std::bind(&K100MotionPlanningNode::planPoseGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_
    );

    RCLCPP_INFO(this->get_logger(), "========== K100 Motion Planning Node 初始化完成 ==========");
    return true;
}

void K100MotionPlanningNode::waitForJointStates() {
    RCLCPP_INFO(this->get_logger(), "等待 joint_states 数据就绪...");
    bool joint_states_received = false;
    auto joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [&joint_states_received](const sensor_msgs::msg::JointState::SharedPtr) {
            joint_states_received = true;
        });
    
    auto wait_start = std::chrono::steady_clock::now();
    while (!joint_states_received && rclcpp::ok()) {
        if (std::chrono::steady_clock::now() - wait_start > std::chrono::seconds(10)) {
            RCLCPP_WARN(this->get_logger(), "等待 joint_states 超时");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (joint_states_received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(this->get_logger(), "joint_states 数据已就绪");
    }
}

bool K100MotionPlanningNode::switchControllers(const std::string& target_group) {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (target_group == "both_arms") {
        activate_controllers = {"both_arms_controller"};
        deactivate_controllers = {"left_arm_controller", "right_arm_controller"};
    } else if (target_group == "left_arm" || target_group == "right_arm") {
        activate_controllers = {"left_arm_controller", "right_arm_controller"};
        deactivate_controllers = {"both_arms_controller"};
    } else {
        return true;
    }

    if (!switch_controller_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "switch_controller 服务不可用");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "尝试切换控制器: Activate=[%s], Deactivate=[%s]", 
        activate_controllers.empty() ? "" : activate_controllers[0].c_str(),
        deactivate_controllers.empty() ? "" : deactivate_controllers[0].c_str());

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = activate_controllers;
    request->deactivate_controllers = deactivate_controllers;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

    auto future = switch_controller_client_->async_send_request(request);
    
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        RCLCPP_ERROR(this->get_logger(), "切换控制器超时");
        return false;
    }

    auto response = future.get();
    if (!response->ok) {
        RCLCPP_ERROR(this->get_logger(), "切换控制器失败");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "控制器切换成功");
    return true;
}

std::shared_ptr<MotionPlanning> K100MotionPlanningNode::getPlanner(const std::string& group_name) {
    if (!switchControllers(group_name)) {
        RCLCPP_ERROR(this->get_logger(), "控制器切换失败，无法获取规划器");
        return nullptr;
    }

    if (group_name == "left_arm") return left_planner_;
    if (group_name == "right_arm") return right_planner_;
    if (group_name == "both_arms") return both_arms_planner_;
    return nullptr;
}

void K100MotionPlanningNode::planJointGoalCallback(const std::shared_ptr<k100_motion_planning::srv::PlanJointGoal::Request> request,
                           std::shared_ptr<k100_motion_planning::srv::PlanJointGoal::Response> response) {
    RCLCPP_INFO(this->get_logger(), "收到 PlanJointGoal 请求: Group=%s", request->group_name.c_str());
    
    auto planner = getPlanner(request->group_name);
    if (!planner) {
        response->success = false;
        response->message = "未知的规划组: " + request->group_name;
        return;
    }

    std::vector<std::vector<double>> waypoints = {request->joint_angles};
    auto plan = planner->planJointGoal(waypoints);
    
    if (plan.trajectory_.joint_trajectory.points.empty()) {
        response->success = false;
        response->message = "规划失败";
    } else {
        bool exec_success = planner->executePlan(plan);
        response->success = exec_success;
        response->message = exec_success ? "执行成功" : "执行失败";
    }
}

void K100MotionPlanningNode::computeIKCallback(const std::shared_ptr<k100_motion_planning::srv::ComputeIK::Request> request,
                       std::shared_ptr<k100_motion_planning::srv::ComputeIK::Response> response) {
    RCLCPP_INFO(this->get_logger(), "收到 ComputeIK 请求: Group=%s", request->group_name.c_str());
    
    auto planner = getPlanner(request->group_name);
    if (!planner) {
        response->success = false;
        response->message = "未知的规划组: " + request->group_name;
        return;
    }

    auto joints = planner->computeIKArray(request->pose);
    if (joints.empty()) {
        response->success = false;
        response->message = "IK 求解失败";
    } else {
        response->success = true;
        response->joint_angles = joints;
        response->message = "IK 求解成功";
    }
}

void K100MotionPlanningNode::planPoseGoalCallback(const std::shared_ptr<k100_motion_planning::srv::PlanPoseGoal::Request> request,
                          std::shared_ptr<k100_motion_planning::srv::PlanPoseGoal::Response> response) {
    RCLCPP_INFO(this->get_logger(), "收到 PlanPoseGoal 请求: Group=%s", request->group_name.c_str());
    
    auto planner = getPlanner(request->group_name);
    if (!planner) {
        response->success = false;
        response->message = "未知的规划组: " + request->group_name;
        return;
    }

    auto plan = planner->planPoseGoal(request->pose, request->ee_link);
    
    if (plan.trajectory_.joint_trajectory.points.empty()) {
        response->success = false;
        response->message = "规划失败";
    } else {
        bool exec_success = planner->executePlan(plan);
        response->success = exec_success;
        response->message = exec_success ? "执行成功" : "执行失败";
    }
}


void K100MotionPlanningNode::playRosbagCallback(const std::shared_ptr<k100_motion_planning::srv::PlayRosbag::Request> request,
                        std::shared_ptr<k100_motion_planning::srv::PlayRosbag::Response> response) {
    RCLCPP_INFO(this->get_logger(), "收到 PlayRosbag 请求: File=%s, Group=%s", request->file_path.c_str(), request->group_name.c_str());

    auto planner = getPlanner(request->group_name);
    if (!planner) {
        response->success = false;
        response->message = "未知的规划组: " + request->group_name;
        return;
    }

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = request->file_path;
    storage_options.storage_id = "sqlite3"; // Default storage id

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader;
    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("无法打开 rosbag 文件: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    rclcpp::Serialization<sensor_msgs::msg::JointState> serialization;
    
    auto target_joint_names = planner->moveGroup()->getJointNames();
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = target_joint_names;

    int64_t start_time_ns = -1;
    size_t point_count = 0;

    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        if (bag_message->topic_name == "/joint_states") {
            sensor_msgs::msg::JointState msg;
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&serialized_msg, &msg);

            trajectory_msgs::msg::JointTrajectoryPoint point;
            int64_t current_time_ns = bag_message->time_stamp;
            if (start_time_ns < 0) start_time_ns = current_time_ns;
            
            point.time_from_start = rclcpp::Duration::from_nanoseconds(current_time_ns - start_time_ns);

            bool all_joints_found = true;
            for (const auto& joint_name : target_joint_names) {
                bool found = false;
                for (size_t i = 0; i < msg.name.size(); ++i) {
                    if (msg.name[i] == joint_name) {
                        point.positions.push_back(msg.position[i]);
                        if (msg.velocity.size() > i) point.velocities.push_back(msg.velocity[i]);
                        if (msg.effort.size() > i) point.accelerations.push_back(msg.effort[i]); 
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    all_joints_found = false;
                    break;
                }
            }
            
            if (all_joints_found) {
                trajectory.joint_trajectory.points.push_back(point);
                point_count++;
            }
        }
    }

    if (point_count == 0) {
        response->success = false;
        response->message = "未找到匹配的 joint_states 数据或数据为空";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "提取到 %zu 个轨迹点", point_count);
    
    // 获取当前机器人状态
    auto state = planner->moveGroup()->getCurrentState(5.0);
    if (!state) {
        response->success = false;
        response->message = "无法获取当前机器人状态";
        return;
    }
    
    // 检查当前位置与轨迹起点的差距
    std::vector<double> first_point_positions = trajectory.joint_trajectory.points[0].positions;
    std::vector<double> current_positions;
    state->copyJointGroupPositions(request->group_name, current_positions);
    
    double max_diff = 0.0;
    size_t max_diff_joint_idx = 0;
    for (size_t i = 0; i < current_positions.size() && i < first_point_positions.size(); ++i) {
        double diff = std::abs(current_positions[i] - first_point_positions[i]);
        if (diff > max_diff) {
            max_diff = diff;
            max_diff_joint_idx = i;
        }
    }
    
    const double threshold = 0.1; // 约5.7度
    if (max_diff > threshold) {
        std::string joint_name = (max_diff_joint_idx < target_joint_names.size()) ? 
            target_joint_names[max_diff_joint_idx] : "unknown";
        RCLCPP_WARN(this->get_logger(), 
            "警告: 当前位置与轨迹起点差距较大! 最大差值关节: %s, 差值: %.4f rad (%.2f deg)", 
            joint_name.c_str(), max_diff, max_diff * 180.0 / M_PI);
        RCLCPP_WARN(this->get_logger(), 
            "当前值: %.4f, 轨迹起点值: %.4f", 
            current_positions[max_diff_joint_idx], first_point_positions[max_diff_joint_idx]);
    }
    
    robot_trajectory::RobotTrajectory robot_traj(state->getRobotModel(), request->group_name);
    robot_traj.setRobotTrajectoryMsg(*state, trajectory);
    
    // 根据参数决定是否进行时间参数化
    if (request->enable_time_parameterization) {
        RCLCPP_INFO(this->get_logger(), "开启时间参数优化 (TimeOptimalTrajectoryGeneration)");
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        double velocity_scaling = 0.5;
        double acceleration_scaling = 0.5;
        
        if (!totg.computeTimeStamps(robot_traj, velocity_scaling, acceleration_scaling)) {
            response->success = false;
            response->message = "轨迹时间参数化失败";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "时间参数化后轨迹点数: %zu", robot_traj.getWayPointCount());
    } else {
        RCLCPP_INFO(this->get_logger(), "跳过时间参数优化，使用原始轨迹时间");
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    robot_traj.getRobotTrajectoryMsg(plan.trajectory_);
    plan.start_state_ = moveit_msgs::msg::RobotState();
    
    RCLCPP_INFO(this->get_logger(), "最终轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
    
    if (planner->executePlan(plan)) {
        response->success = true;
        response->message = "执行成功";
    } else {
        response->success = false;
        response->message = "执行失败";
    }
}

} // namespace k100_motion_planning
