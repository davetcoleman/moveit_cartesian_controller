/* Copyright 2015 Google Inc.
   Author: Dave Coleman <dave@dav.ee>
   Desc:   Quickly convert a desired end effector pose into a joint trajectory
           that follows a smooth cartesian path and send directly to ros_control
           controllers with little to no latency. Realtime capabailities for
           MoveIt!
*/

#ifndef CARTESIAN_CONTROLLER_CART_PATH_PLANNER_H
#define CARTESIAN_CONTROLLER_CART_PATH_PLANNER_H

// C++
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

// Boost
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// moveit_boilerplate
#include <moveit_boilerplate/moveit_base.h>

// MoveIt!
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene/planning_scene.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_cartesian_controller
{

MOVEIT_CLASS_FORWARD(CartPathPlanner);

typedef std::function<void(const moveit::core::RobotState &)> VisualizeStateCallback;
typedef std::function<void(const trajectory_msgs::JointTrajectory &)> TrajectoryCommandCallback;

class CartPathPlanner : public moveit_boilerplate::MoveItBase
{
public:
  /** \brief Constructor */
  CartPathPlanner();

  /** \brief Destructor */
  virtual ~CartPathPlanner();

  /** \brief Inititalize the planner */
  bool init(ros::NodeHandle &nh);

  /** \brief Callback to allow this class to command joint trajectories*/
  void setTrajectoryCommandCallback(TrajectoryCommandCallback trajectory_command_callback)
  {
    trajectory_command_callback_ = trajectory_command_callback;
  }

  /** \brief Set how many interations */
  void setTimeParamIterations(double time_param_iterations);

  /** \brief Callback from recieving new robot state
             This version of the function is used for the
             moveit_cartesian_controller / joint_trajectory_controller
  */
  void controllerStateCB(const std::vector<double>& future_positions,
                         const std::vector<double>& future_velocities,
                         const std::vector<double>& future_accelerations,
                         const ros::Time& future_time);

  /** \brief Callback from recieving new robot state
             This version of the function is used for Reflexxes
   */
  void controllerStateCB(const std::vector<double>& future_positions,
                         const std::vector<double>& future_velocities);

  /** \brief Callback from ROS cartesian command for desired pose of end effector */
  void cartCommandCB(const geometry_msgs::PoseStamped::ConstPtr &command);

  /** \brief Enable test mode */
  void enableTestMode()
  {
    use_test_trajectory_ = true;
  }

  /** \brief Do not show the robot state in rviz */
  void hideRobotState()
  {
    hide_robot_state_ = true;
  }

  /** \brief Get the average amount of time IK planner takes */
  double getAveragePlanTime()
  {
    return average_plan_time_;
  }

private:

  /** \brief Publish the robot state to Rviz in a separate thread */
  void visualizationThread(const ros::TimerEvent &e);

  /** \brief Main planning thread for computing paths */
  void planThread();

  /** \brief Main processing pipeline for solving IK and executing on hardware */
  void cartPathPipeline();

  /**
   * \brief Quickly response to pose requests. Uses IK on dev computer, not embedded
   * \return true on success
   */
  bool solveIK();

  /** \brief Test function */
  void computeTestTrajectory(const moveit::core::RobotStatePtr start_state,
                             std::vector<moveit::core::RobotStatePtr> &cart_traj);

  /** \brief In separate thread from imarker & Ik solver, send joint commands */
  void commandJoints();

  /**
   * \brief Compute a cartesian path along waypoints
   * \return true on success
   */
  bool computeCartWaypointPath(const moveit::core::RobotStatePtr start_state, const EigenSTL::vector_Affine3d &waypoints,
                               std::vector<moveit::core::RobotStatePtr> &cart_traj);

  /**
   * \brief Convert and parameterize a trajectory with velocity information utilize
   * \return true on success
   */
  bool convertRobotStatesToTraj(const std::vector<moveit::core::RobotStatePtr> &robot_state_traj,
                                      moveit_msgs::RobotTrajectory &trajectory_msg);

  /** \brief Update a robot state based on the desired controller state */
  //void controllerStateToRobotState(moveit::core::RobotStatePtr &robot_state);

  /**
   * \brief Compare two Eigen poses
   * \return true if poses are within epsilon
   */
  bool posesEqual(const Eigen::Affine3d &pose1, const Eigen::Affine3d &pose2);

  // --------------------------------------------------------
  // Name of this class
  std::string name_ = "cart_path_planner";

  // Planning thread
  std::thread plan_thread_;
  std::mutex plan_thread_mutex_;
  std::condition_variable plan_thread_condition_var_;
  bool plan_thread_processing_ = false;

  // Command trajectory
  TrajectoryCommandCallback trajectory_command_callback_;

  // Desired planning group to work with
  JointModelGroup *arm_jmg_;

  // Flag to determine if new desired pose needs to be processed
  volatile bool has_pose_to_ik_solve_ = false;

  // Debug values
  bool debug_command_rate_ = false;
  bool debug_generated_traj_rate_ = false;
  bool debug_show_time_parameterization_ = false;
  bool debug_show_trajectory_ = false;
  bool debug_save_generated_trajectory_;

  // Subscriber to cartesian commands
  ros::Subscriber cmd_sub_;

  // Desired end effector pose
  Eigen::Affine3d desired_ee_pose_;
  boost::shared_mutex desired_ee_pose_mutex_;

  // Parameter for when two poses are considered equal
  double pose_equal_threshold_;
  double average_plan_time_;

  // State to start cartesian planning from
  moveit::core::RobotStatePtr start_planning_state_;
  ros::Time start_planning_time_; // when the robot will be at the start_planning_state_

  // Cartesian Inverse Kinematics -------------------------------
  // Maintain robot state at interactive marker (not the current robot state)
  const moveit::core::LinkModel *ik_tip_link_;
  EigenSTL::vector_Affine3d cart_desired_waypoints_;

  // IK Settings
  double ik_consistency_limit_;
  double ik_cart_max_step_;        // Resolution of trajectory, the maximum distance in Cartesian space
                                   // between consecutive points on the resulting path
  double ik_cart_jump_threshold_;  // Threshold for preventing consequtive joint
                                   // values from 'jumping' by a large amount in joint space
  double vel_scaling_factor_;      // 0-1
  std::vector<double> ik_consistency_limits_vector_;

  // Joint Command -----------------------------------
  moveit_msgs::RobotTrajectory trajectory_msg_;

  // Stats
  std::size_t total_commands_ = 0;
  ros::Duration total_command_duration_;

  // Tool for parameterizing trajectories with velocities and accelerations
  boost::shared_ptr<trajectory_processing::IterativeParabolicTimeParameterization> iterative_smoother_;

  // Collision Checking ---------------------------------------------
  bool use_collision_checking_;

  // Test
  bool use_test_trajectory_ = false;

  // Visualization -------------------------------------
  bool has_state_to_visualize_ = false;
  double visualization_rate_;  // hz
  ros::Timer non_realtime_loop_;
  bool hide_robot_state_ = false;

  // Saving to file ------------------------------------
  double trajectory_filename_count_ = 0;
  std::string save_traj_to_file_path_;

};  // end class

}  // namespace moveit_cartesian_controller

namespace
{
/** \brief Collision checking handle for IK solvers */
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose, bool only_check_self_collision,
                  mvt::MoveItVisualToolsPtr visual_tools_, robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, const double *ik_solution);
}

#endif  // CARTESIAN_CONTROLLER_CART_PATH_PLANNER_H
