#ifndef K100_MOTION_PLANNING__K100_MOTION_PLANNING_HPP_
#define K100_MOTION_PLANNING__K100_MOTION_PLANNING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include "k100_motion_planning/motion_planning.hpp"
#include "k100_motion_planning/srv/plan_joint_goal.hpp"
#include "k100_motion_planning/srv/play_rosbag.hpp"
#include "k100_motion_planning/srv/compute_ik.hpp"
#include "k100_motion_planning/srv/plan_pose_goal.hpp"

namespace k100_motion_planning {

class K100MotionPlanningNode : public rclcpp::Node {
public:
    K100MotionPlanningNode();
    bool initialize();

private:
    std::shared_ptr<MotionPlanning> left_planner_;
    std::shared_ptr<MotionPlanning> right_planner_;
    std::shared_ptr<MotionPlanning> both_arms_planner_;
    
    rclcpp::Service<k100_motion_planning::srv::PlanJointGoal>::SharedPtr plan_joint_goal_srv_;
    rclcpp::Service<k100_motion_planning::srv::PlayRosbag>::SharedPtr play_rosbag_srv_;
    rclcpp::Service<k100_motion_planning::srv::ComputeIK>::SharedPtr compute_ik_srv_;
    rclcpp::Service<k100_motion_planning::srv::PlanPoseGoal>::SharedPtr plan_pose_goal_srv_;
    
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    void waitForJointStates();
    std::shared_ptr<MotionPlanning> getPlanner(const std::string& group_name);
    bool switchControllers(const std::string& target_group);

    void planJointGoalCallback(const std::shared_ptr<k100_motion_planning::srv::PlanJointGoal::Request> request,
                               std::shared_ptr<k100_motion_planning::srv::PlanJointGoal::Response> response);

    void playRosbagCallback(const std::shared_ptr<k100_motion_planning::srv::PlayRosbag::Request> request,
                            std::shared_ptr<k100_motion_planning::srv::PlayRosbag::Response> response);

    void computeIKCallback(const std::shared_ptr<k100_motion_planning::srv::ComputeIK::Request> request,
                           std::shared_ptr<k100_motion_planning::srv::ComputeIK::Response> response);

    void planPoseGoalCallback(const std::shared_ptr<k100_motion_planning::srv::PlanPoseGoal::Request> request,
                              std::shared_ptr<k100_motion_planning::srv::PlanPoseGoal::Response> response);
};

} // namespace k100_motion_planning

#endif // K100_MOTION_PLANNING__K100_MOTION_PLANNING_HPP_
