/* Copyright 2015 Google Inc.
   Author: Dave Coleman <dave@dav.ee>
   Desc:   CartPathPlanner
*/

#include <moveit_cartesian_controller/cart_path_planner.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// ROS Boilerplate
#include <moveit_boilerplate/execution_interface.h>

namespace moveit_cartesian_controller
{

CartPathPlanner::CartPathPlanner()
  : moveit_boilerplate::MoveItBase()
{
}

bool CartPathPlanner::init(ros::NodeHandle &nh)
{
  // Copy the namespace of our node
  nh_ = nh;

  // Load the MoveIt components
  MoveItBase::init(nh_);

  // Rename joint model group
  arm_jmg_ = jmg_;

  // Parameters
  double time_param_iterations;

  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  int error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "ik_consistency_limit", ik_consistency_limit_);
  error += !rosparam_shortcuts::get(name_, rpnh, "ik_cart_max_step", ik_cart_max_step_);
  error += !rosparam_shortcuts::get(name_, rpnh, "ik_cart_jump_threshold", ik_cart_jump_threshold_);
  error += !rosparam_shortcuts::get(name_, rpnh, "vel_scaling_factor", vel_scaling_factor_);
  error += !rosparam_shortcuts::get(name_, rpnh, "time_param_iterations", time_param_iterations);
  error += !rosparam_shortcuts::get(name_, rpnh, "pose_equal_threshold", pose_equal_threshold_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualization_rate", visualization_rate_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_collision_checking", use_collision_checking_);
  error += !rosparam_shortcuts::get(name_, rpnh, "average_plan_time", average_plan_time_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/command_rate", debug_command_rate_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/generated_traj_rate", debug_generated_traj_rate_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/show_time_parameterization", debug_show_time_parameterization_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/show_trajectory", debug_show_trajectory_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/save_generated_trajectory", debug_save_generated_trajectory_);
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/save_traj_to_file_path", save_traj_to_file_path_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Warn user if no collision checking
  if (!use_collision_checking_)
    ROS_WARN_STREAM_NAMED(name_, "Not using collision checking because disabled in yaml config");

  // Load the parameterizer
  setTimeParamIterations(time_param_iterations);

  // Create consistency limits for IK solving. Pre-build this vector for improved speed
  // This specifies the desired distance between the solution and the seed state
  if (ik_consistency_limit_)
    for (std::size_t i = 0; i < arm_jmg_->getActiveJointModels().size(); ++i)
      ik_consistency_limits_vector_.push_back(ik_consistency_limit_);

  // End effector parent link (arm tip for ik solving)
  ik_tip_link_ = arm_jmg_->getOnlyOneEndEffectorTip();
  if (!ik_tip_link_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Did not find ik_tip_link");
  }

  // Compute cartesian command input
  cart_desired_waypoints_.resize(1);

  // Check for kinematic solver
  if (!arm_jmg_->canSetStateFromIK(ik_tip_link_->getName()))
    ROS_ERROR_STREAM_NAMED(name_, "No IK Solver loaded - make sure moveit_config/kinamatics.yaml is loaded in this namespace");

  start_planning_state_.reset(new moveit::core::RobotState(*current_state_));

  // Create thread for computing paths
  ROS_DEBUG_STREAM_NAMED(name_, "Starting planning thread");
  plan_thread_ = std::thread(&CartPathPlanner::planThread, this);

  // Subscribe to commands
  cmd_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("cartesian_command", 1, &CartPathPlanner::cartCommandCB, this);

  // Thread for publishing to rviz
  non_realtime_loop_ =
      nh_.createTimer(ros::Duration(1.0 / visualization_rate_), &CartPathPlanner::visualizationThread, this);

  return true;
}

CartPathPlanner::~CartPathPlanner() {}

void CartPathPlanner::planThread()
{
  //std::cout << "[worker thread] started ------------------------ " << std::endl;
  while (ros::ok())
  {
    // Wait until state data is recieved
    std::unique_lock<std::mutex> lock(plan_thread_mutex_);
    plan_thread_condition_var_.wait(lock, [this]{return plan_thread_processing_;});
    lock.unlock();

    // DO PROCESSING
    //std::cout << "[worker thread] processing data: " << std::endl;

    // Run pipeline
    cartPathPipeline();

    // Send data back to main()
    std::unique_lock<std::mutex> lock2(plan_thread_mutex_);
    plan_thread_processing_ = false;
    //std::cout << "[worker thread] signals data processing completed\n";

    // Manual unlocking is done before notifying, to avoid waking up
    // the waiting thread only to block again (see notify_one for details)
    lock2.unlock();

    plan_thread_condition_var_.notify_one();
  }
  //std::cout << "[worker thread] exiting " << std::endl;
}

void CartPathPlanner::cartCommandCB(const geometry_msgs::PoseStamped::ConstPtr &command)
{
  // Convert to eigen
  Eigen::Affine3d input_pose;
  tf::poseMsgToEigen(command->pose, input_pose);

  // Get write mutex lock
  {
    boost::lock_guard<boost::shared_mutex> lock(desired_ee_pose_mutex_);

    // Set
    desired_ee_pose_ = input_pose;
  }

  // Mark pose as ready for ik solving
  has_pose_to_ik_solve_ = true;
}

void CartPathPlanner::visualizationThread(const ros::TimerEvent &e)
{
  // Check if there is anything to visualize
  if (has_state_to_visualize_)
  {
    has_state_to_visualize_ = false;
    visual_tools_->publishRobotState(start_planning_state_, rvt::GREEN);
  }
  else if (hide_robot_state_)
  {
    hide_robot_state_ = false;
    visual_tools_->hideRobot();
  }
}

void CartPathPlanner::controllerStateCB(const std::vector<double>& future_positions,
                                        const std::vector<double>& future_velocities,
                                        const std::vector<double>& future_accelerations,
                                        const ros::Time& future_time)
{
  // Check if we should process this state
  {
    std::unique_lock<std::mutex> lock(plan_thread_mutex_);
    if (plan_thread_processing_)
    {
      //ROS_WARN_STREAM_THROTTLE_NAMED(1, name_, "Plan thread not ready for next state");
      return;
    }
  }

  // Error check
  if (future_positions.size() != future_velocities.size())
  {
    ROS_FATAL_STREAM_NAMED(name_, "Position and velocity vectors do not match in size");
    exit(-1);
  }
  if (future_velocities.size() != future_accelerations.size())
  {
    ROS_FATAL_STREAM_NAMED(name_, "Velocity and acceleration vectors do not match in size");
    exit(-1);
  }
  if (future_positions.size() != jmg_->getActiveJointModels().size())
  {
    ROS_FATAL_STREAM_NAMED(name_, "Controller future state has " << future_positions.size() << " joints but "
                           << " robot model requires " << jmg_->getActiveJointModels().size());
    exit(-1);
  }

  // Copy values
  start_planning_state_->setJointGroupPositions(jmg_, future_positions);
  start_planning_state_->setJointGroupVelocities(jmg_, future_velocities);
  start_planning_state_->setJointGroupAccelerations(jmg_, future_accelerations);
  start_planning_time_ = future_time;

  // Process latest state
  {
    std::lock_guard<std::mutex> lock(plan_thread_mutex_);
    plan_thread_processing_ = true;
    //std::cout << "[loop] State loop signals data ready for processing\n";
  }
  plan_thread_condition_var_.notify_one();
}

void CartPathPlanner::controllerStateCB(const std::vector<double>& future_positions,
                                        const std::vector<double>& future_velocities)
{
  // Check if we should process this state
  {
    std::unique_lock<std::mutex> lock(plan_thread_mutex_);
    if (plan_thread_processing_)
    {
      //ROS_WARN_STREAM_THROTTLE_NAMED(1, name_, "Plan thread not ready for next state");
      return;
    }
  }

  // Error check
  if (future_positions.size() != future_velocities.size())
  {
    ROS_FATAL_STREAM_NAMED(name_, "Position and velocity vectors do not match in size");
    exit(-1);
  }
  if (future_positions.size() != jmg_->getActiveJointModels().size())
  {
    ROS_FATAL_STREAM_NAMED(name_, "Controller future state has " << future_positions.size() << " joints but "
                           << " robot model requires " << jmg_->getActiveJointModels().size());
    exit(-1);
  }

  // Copy values
  start_planning_state_->setJointGroupPositions(jmg_, future_positions);
  start_planning_state_->setJointGroupVelocities(jmg_, future_velocities);

  // Process latest state
  {
    std::lock_guard<std::mutex> lock(plan_thread_mutex_);
    plan_thread_processing_ = true;
    //std::cout << "[loop] State loop signals data ready for processing\n";
  }
  plan_thread_condition_var_.notify_one();
}

void CartPathPlanner::cartPathPipeline()
{
  if (!has_pose_to_ik_solve_)
  {
    //ROS_WARN_STREAM_THROTTLE_NAMED(1, name_, "No pose to ik solve");
    return;
  }

  // Optionally start timer
  ros::Time begin_time;
  if (debug_command_rate_)
    begin_time = ros::Time::now();

  // Publish current state
  has_state_to_visualize_ = true;
  hide_robot_state_ = false;

  // Plan IK Path
  if (!solveIK())
  {
    has_pose_to_ik_solve_ = false;
    return;
  }

  // Send trajectory for execution
  commandJoints();

  // Optionally end timer
  if (debug_command_rate_)
  {
    ros::Duration duration = (ros::Time::now() - begin_time);
    total_command_duration_ += duration;
    total_commands_++;

    double average_duration = total_command_duration_.toSec() / total_commands_;
    ROS_INFO_STREAM_THROTTLE_NAMED(5, name_,
                                   "Cmnd duration: " << std::fixed << duration.toSec() << "\t avg: " << std::fixed
                                   << average_duration << "\t avg freq: " << std::fixed
                                   << (1.0 / average_duration) << " hz");
  }

  // Set the planner as ready
  has_pose_to_ik_solve_ = false;
}

bool CartPathPlanner::solveIK()
{
  // Get read-only mutex lock
  {
    boost::shared_lock<boost::shared_mutex> lock(desired_ee_pose_mutex_);
    // Command input pose
    cart_desired_waypoints_.front() = desired_ee_pose_;
  }

  // Check if the desired pose is the same as our current pose
  if (posesEqual(start_planning_state_->getGlobalLinkTransform(ik_tip_link_), cart_desired_waypoints_.front()))
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Robot already at desired pose, ignoring command");
    return false;
  }

  std::vector<moveit::core::RobotStatePtr> cart_traj;

  // Possibly run test
  if (!use_test_trajectory_)
  {
    // Plan smooth cartesian path using IK only
    if (!computeCartWaypointPath(start_planning_state_, cart_desired_waypoints_, cart_traj))
    {
    ROS_DEBUG_STREAM_NAMED(name_, "Unable to compute cartesian path");
      return false;
    }

    // Replace first RobotState with one that has current velocity and accelerations
    cart_traj.front() = start_planning_state_;
  }
  else
  {
    // Plan a dummy trajectory
    computeTestTrajectory(start_planning_state_, cart_traj);
  }

  // Get trajectory message
  if (!convertRobotStatesToTraj(cart_traj, trajectory_msg_))
  {
    ROS_INFO_STREAM_NAMED(name_, "Failed to convert to parameterized trajectory");
    return false;
  }

  // Statistics on generated trajectory
  if (debug_generated_traj_rate_)
  {
    const trajectory_msgs::JointTrajectory &traj = trajectory_msg_.joint_trajectory;
    double avg_freq = 1.0 / (traj.points.back().time_from_start.toSec() / double(traj.points.size() - 2));
    ROS_INFO_STREAM_NAMED(name_, "Avg frequency of generated trajectory waypoints: " << avg_freq << " hz");
  }

  // Save generated trajectory
  if (debug_save_generated_trajectory_)
  {
    // Only save first one
    if (trajectory_filename_count_ == 0)
    {
      moveit_boilerplate::ExecutionInterface::saveTrajectory(trajectory_msg_, jmg_->getName() + "_moveit_trajectory_"
                                                             + boost::lexical_cast<std::string>(trajectory_filename_count_++) + ".csv",
                                                             save_traj_to_file_path_);
      trajectory_filename_count_++;
    }
    else
      ROS_INFO_STREAM_NAMED(name_, "Not saving trajectory after first one");
  }

  return true;
}

void CartPathPlanner::computeTestTrajectory(const moveit::core::RobotStatePtr start_state,
                                            std::vector<moveit::core::RobotStatePtr> &cart_traj)
{
  ROS_ERROR_STREAM_NAMED(name_, "Computing test trajectory!");

  // Default all 0's
  //start_state->setToDefaultValues();

  // Add current state
  moveit::core::RobotStatePtr s1(new moveit::core::RobotState(*start_state));
  cart_traj.push_back(start_state);

  // Resusable positions
  std::vector<double> position90;
  position90.push_back(1.57);
  std::vector<double> position45;
  position45.push_back(1.57 / 2.0);
  std::vector<double> positionN45;
  positionN45.push_back(-1.57 / 2.0);
  std::vector<double> position30;
  position30.push_back(0.523599);
  std::vector<double> positionN30;
  positionN30.push_back(-0.523599);
  std::vector<double> position0;
  position0.push_back(0.0);

  // Twist one joint
  moveit::core::RobotStatePtr s2(new moveit::core::RobotState(*s1));
  s2->setJointPositions("joint_a6", position90);
  s2->setJointPositions("joint_a5", position45);
  s2->setJointPositions("joint_a4", position0);
  s2->setJointPositions("joint_a3", position0);
  s2->setJointPositions("joint_a2", position30);
  s2->setJointPositions("joint_a1", position45);
  cart_traj.push_back(s2);

  // Twist another joint
  moveit::core::RobotStatePtr s3(new moveit::core::RobotState(*s2));
  s3->setJointPositions("joint_a6", position0);
  s3->setJointPositions("joint_a5", position90);
  s3->setJointPositions("joint_a4", position45);
  s3->setJointPositions("joint_a3", position0);
  s3->setJointPositions("joint_a2", position0);
  s3->setJointPositions("joint_a1", position90);
  cart_traj.push_back(s3);

  // Twist another joint
  moveit::core::RobotStatePtr s4(new moveit::core::RobotState(*s3));
  s4->setJointPositions("joint_a6", position90);
  s4->setJointPositions("joint_a5", position90);
  s4->setJointPositions("joint_a4", position90);
  s4->setJointPositions("joint_a3", position0);
  s4->setJointPositions("joint_a2", positionN30);
  s4->setJointPositions("joint_a1", position90);
  cart_traj.push_back(s4);

  // Twist another joint
  moveit::core::RobotStatePtr s5(new moveit::core::RobotState(*s3));
  s5->setJointPositions("joint_a6", position0);
  s5->setJointPositions("joint_a5", position45);
  s5->setJointPositions("joint_a4", position45);
  s5->setJointPositions("joint_a3", position0);
  s5->setJointPositions("joint_a2", position0);
  s5->setJointPositions("joint_a1", position45);
  cart_traj.push_back(s5);

  // Return to original state
  moveit::core::RobotStatePtr s6(new moveit::core::RobotState(*start_state));
  cart_traj.push_back(s6);

  //ROS_INFO_STREAM_NAMED(name_, "Cartesian trajectory has size " << cart_traj.size());
}

void CartPathPlanner::commandJoints()
{
  trajectory_msgs::JointTrajectory &trajectory = trajectory_msg_.joint_trajectory;

  // Remove first point
  //trajectory.points.erase(trajectory.points.begin());
  //trajectory.points.begin()->velocities.erase(trajectory.points.begin()->velocities.begin());

  // Add timestamp to trajectory
  trajectory.header.stamp = start_planning_time_;

  // Debug timestamp
  double time_in_future = (start_planning_time_ - ros::Time::now()).toSec();
  if (time_in_future < 0)
    ROS_DEBUG_STREAM_NAMED(name_, "Plan scheduled to occur in past: " << time_in_future);

  // Debug
  if (debug_show_trajectory_)
  {
    std::cout << "Sending commanded trajectory: \n" << trajectory_msg_ << std::endl;
  }

  // Error check
  if (trajectory.points.empty())
  {
    ROS_ERROR_STREAM_NAMED(name_, "No points left in trajectory");
    return;
  }

  // Execute
  trajectory_command_callback_(trajectory);
}

bool CartPathPlanner::computeCartWaypointPath(const moveit::core::RobotStatePtr start_state,
                                              const EigenSTL::vector_Affine3d &waypoints,
                                              std::vector<moveit::core::RobotStatePtr> &cart_traj)
{
  // Cartesian settings
  const bool collision_checking_verbose = false;
  const bool only_check_self_collision = false;
  const bool global_reference_frame = true;  // Reference frame setting

  // Results
  double last_valid_percentage;

  // Create a temp state that doesn't have velocity or accelerations
  moveit::core::RobotState cart_state(*start_state);

  // Clear both the velocities and accelerations because this state is copied
  // for the cartesian path but the vel & accel are left the original (incorrect) values
  std::vector<double> zero_variables(cart_state.getVariableCount(), 0.0);
  cart_state.setVariableVelocities(zero_variables);
  cart_state.setVariableAccelerations(zero_variables);

  // Optionally collision check
  moveit::core::GroupStateValidityCallbackFn constraint_fn;
  if (use_collision_checking_)
  {
    boost::scoped_ptr<psm::LockedPlanningSceneRO> ls;
    ls.reset(new psm::LockedPlanningSceneRO(planning_scene_monitor_));
    constraint_fn = boost::bind(&isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr &>(*ls).get(),
                                collision_checking_verbose, only_check_self_collision, visual_tools_, _1, _2, _3);
  }

  // Compute Cartesian Path
  cart_traj.clear();
  last_valid_percentage = cart_state.computeCartesianPath(arm_jmg_, cart_traj, ik_tip_link_, waypoints,
                                                          global_reference_frame, ik_cart_max_step_,
                                                          ik_cart_jump_threshold_, constraint_fn,
                                                          kinematics::KinematicsQueryOptions());

  const double min_valid_percentage = 0.05;
  if (last_valid_percentage < min_valid_percentage)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Cartesian last_valid_percentage: " << last_valid_percentage
                           << " number of points: " << cart_traj.size()
                           << " failed to compute path");
    return false;
  }

  ROS_DEBUG_STREAM_NAMED(name_, "Cartesian last_valid_percentage: " << last_valid_percentage
                         << " number of points: " << cart_traj.size());

  // Delete waypoints that are the same
  bool is_diff;
  //std::cout << "Num waypoints: " << cart_traj.size() << std::endl;
  for (std::size_t i = 1; i < cart_traj.size(); ++i)
  {
    const moveit::core::RobotStatePtr &prev_state = cart_traj[i-1];
    const moveit::core::RobotStatePtr &curr_state = cart_traj[i];

    // Check joints inside two states for similarity
    is_diff = false;
    for (std::size_t i = 0; i < prev_state->getVariableCount(); ++i)
    {
      if (prev_state->getVariablePositions()[i] != curr_state->getVariablePositions()[i])
      {
        is_diff = true;
        break;
      }
    }
    if (!is_diff)
    {
      //ROS_WARN_STREAM_NAMED(name_, "Delete state " << i);
      cart_traj.erase(cart_traj.begin()+i);
      i -= 1;
    }
  }
  //std::cout << "Num waypoints after deleting: " << cart_traj.size() << std::endl;

  return true;
}

bool CartPathPlanner::convertRobotStatesToTraj(const std::vector<moveit::core::RobotStatePtr> &robot_state_traj,
                                               moveit_msgs::RobotTrajectory &trajectory_msg)
{
  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_traj(new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg_));

  // Convert to RobotTrajectory datatype
  for (std::size_t k = 0; k < robot_state_traj.size(); ++k)
  {
    double duration_from_previous = 1;  // this is overwritten and unimportant
    robot_traj->addSuffixWayPoint(robot_state_traj[k], duration_from_previous);
  }

  static const std::size_t MIN_TRAJECTORY_POINTS = 0;
  if (robot_traj->getWayPointCount() < MIN_TRAJECTORY_POINTS)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Interpolating trajectory because two few points ("
                           << robot_traj->getWayPointCount() << ")");

    // Interpolate between each point
    double discretization = 0.25;
    double dummy_dt = 1;  // dummy value until parameterization

    robot_trajectory::RobotTrajectoryPtr new_robot_traj(
                                                        new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg_));
    std::size_t original_num_waypoints = robot_traj->getWayPointCount();

    // Error check
    if (robot_traj->getWayPointCount() < 2)
    {
      ROS_WARN_STREAM_NAMED(name_, "Unable to interpolate between less than two states");
      return false;
    }

    // For each set of points (A,B) in the original trajectory
    for (std::size_t i = 0; i < robot_traj->getWayPointCount() - 1; ++i)
    {
      // Add point A to final trajectory
      new_robot_traj->addSuffixWayPoint(robot_traj->getWayPoint(i), dummy_dt);

      for (double t = discretization; t < 1; t += discretization)
      {
        // Create new state
        moveit::core::RobotStatePtr interpolated_state(new moveit::core::RobotState(robot_traj->getFirstWayPoint()));
        // Fill in new values
        robot_traj->getWayPoint(i).interpolate(robot_traj->getWayPoint(i + 1), t, *interpolated_state);
        // Add to trajectory
        new_robot_traj->addSuffixWayPoint(interpolated_state, dummy_dt);
        // std::cout << "inserting " << t << " at " << new_robot_traj->getWayPointCount() <<
        // std::endl;
      }
    }

    // Add final waypoint
    new_robot_traj->addSuffixWayPoint(robot_traj->getLastWayPoint(), dummy_dt);

    std::size_t modified_num_waypoints = new_robot_traj->getWayPointCount();
    ROS_DEBUG_STREAM_NAMED(name_, "Interpolated trajectory from " << original_num_waypoints
                           << " to " << modified_num_waypoints);

    // Copy back to original datastructure
    *robot_traj = *new_robot_traj;
  } // end interpolation

  if (debug_show_time_parameterization_)
  {
    // Convert trajectory to a message
    moveit_msgs::RobotTrajectory trajectory_msg_debug;
    robot_traj->getRobotTrajectoryMsg(trajectory_msg_debug);

    std::cout << "Velocity Before Iterative smoother: " << std::endl;
    for (std::size_t i = 0; i < trajectory_msg_debug.joint_trajectory.points.size(); ++i)
    {
      const std::vector<double>& velocities = trajectory_msg_debug.joint_trajectory.points[i].velocities;
      std::cout << " - Point: " << i << ": ";
      std::copy(velocities.begin(), velocities.end(), std::ostream_iterator<double>(std::cout, ", "));
      std::cout << std::endl;
    }
    std::cout << "Position Before Iterative smoother: " << std::endl;
    for (std::size_t i = 0; i < trajectory_msg.joint_trajectory.points.size(); ++i)
    {
      const std::vector<double>& positions = trajectory_msg_debug.joint_trajectory.points[i].positions;
      std::cout << " - Point: " << i << ": ";
      std::copy(positions.begin(), positions.end(), std::ostream_iterator<double>(std::cout, ", "));
      std::cout << std::endl;
    }
  }

  // Perform iterative parabolic smoothing
  iterative_smoother_->computeTimeStamps(*robot_traj, vel_scaling_factor_);

  // Convert trajectory to a message
  robot_traj->getRobotTrajectoryMsg(trajectory_msg);

  if (debug_show_time_parameterization_)
  {
    std::cout << "Velocity After Iterative smoother: " << std::endl;
    for (std::size_t i = 0; i < trajectory_msg.joint_trajectory.points.size(); ++i)
    {
      const std::vector<double>& velocities = trajectory_msg.joint_trajectory.points[i].velocities;
      std::cout << " - Point: " << i << ": ";
      std::copy(velocities.begin(), velocities.end(), std::ostream_iterator<double>(std::cout, ", "));
      std::cout << std::endl;
    }
    std::cout << "Position After Iterative smoother: " << std::endl;
    for (std::size_t i = 0; i < trajectory_msg.joint_trajectory.points.size(); ++i)
    {
      const std::vector<double>& positions = trajectory_msg.joint_trajectory.points[i].positions;
      std::cout << " - Point: " << i << ": ";
      std::copy(positions.begin(), positions.end(), std::ostream_iterator<double>(std::cout, ", "));
      std::cout << std::endl;
      //trajectory_msg.joint_trajectory.points[i].accelerations.clear();
    }
    std::cout << std::endl;
  }

  return true;
}

bool CartPathPlanner::posesEqual(const Eigen::Affine3d &pose1, const Eigen::Affine3d &pose2)
{
  static const std::size_t NUM_VARS = 16; // size of eigen matrix

  for (std::size_t i = 0; i < NUM_VARS; ++i)
  {
    if (fabs(pose1.data()[i] - pose2.data()[i]) > pose_equal_threshold_)
    {
      return false;
    }
  }

  return true;
}

void CartPathPlanner::setTimeParamIterations(double time_param_iterations)
{
  iterative_smoother_.reset(new trajectory_processing::IterativeParabolicTimeParameterization(time_param_iterations));
}

}  // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose, bool only_check_self_collision,
                  mvt::MoveItVisualToolsPtr visual_tools, moveit::core::RobotState *robot_state, JointModelGroup *group,
                  const double *ik_solution)
{
  // Apply IK solution to robot state
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("cart_path_planner", "No planning scene provided");
    return false;
  }
  if (only_check_self_collision)
  {
    // No easy API exists for only checking self-collision, so we do it here.
    // TODO: move this into planning_scene.cpp
    collision_detection::CollisionRequest req;
    req.verbose = false;
    req.group_name = group->getName();
    collision_detection::CollisionResult res;
    planning_scene->checkSelfCollision(req, res, *robot_state);
    if (!res.collision)
      return true;  // not in collision
  }
  else if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true;  // not in collision

  // Display more info about the collision
  if (verbose)
  {
    visual_tools->publishRobotState(*robot_state, rvt::RED);
    planning_scene->isStateColliding(*robot_state, group->getName(), true);
    visual_tools->publishContactPoints(*robot_state, planning_scene);
    ros::Duration(0.4).sleep();
  }
  ROS_WARN_STREAM_THROTTLE_NAMED(2.0, "cart_path_planner", "Collision");
  return false;
}

}  // end annonymous namespace
