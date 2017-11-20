#pragma once

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <ros/ros.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <set>
#include <thread>
#include <ur_modern_driver/FollowWaypointsAction.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ros/waypoint_follower.h"
#include "ur_modern_driver/ur/consumer.h"
#include "ur_modern_driver/ur/master_board.h"
#include "ur_modern_driver/ur/state.h"

class WPActionServer : public Service
{
private:
  typedef ur_modern_driver::FollowWaypointsAction Action;
  typedef ur_modern_driver::FollowWaypointsResult Result;
  typedef actionlib::ServerGoalHandle<Action> GoalHandle;
  typedef actionlib::ActionServer<Action> Server;

  ros::NodeHandle nh_;
  Server as_;

  double max_velocity_;

  GoalHandle curr_gh_;
  std::atomic<bool> interrupt_traj_;
  std::atomic<bool> has_goal_, running_;
  std::mutex wp_mutex_;
  std::condition_variable wp_cv_;
  std::thread wp_thread_;

  WaypointFollower& wp_follower_;

  RobotState state_;

  void onGoal(GoalHandle gh);
  void onCancel(GoalHandle gh);

  bool validate(GoalHandle& gh, Result& res);
  bool validateState(GoalHandle& gh, Result& res);
  bool validateJoints(GoalHandle& gh, Result& res);
  bool validateTrajectory(GoalHandle& gh, Result& res);

  bool try_execute(GoalHandle& gh, Result& res);
  void interruptGoal(GoalHandle& gh);

  void waypointThread();

public:
  WPActionServer(WaypointFollower& wp_follower, double max_velocity);
  virtual void onRobotStateChange(RobotState state);
  void start();
};
