#include "ur_modern_driver/ros/wp_action_server.h"
#include <cmath>

WPActionServer::WPActionServer(WaypointFollower& wp_follower)
  : as_(nh_, "follow_waypoints", boost::bind(&WPActionServer::onGoal, this, _1),
        boost::bind(&WPActionServer::onCancel, this, _1), false)
  , interrupt_traj_(false)
  , has_goal_(false)
  , running_(false)
  , wp_follower_(wp_follower)
  , state_(RobotState::Error) 
{
}

void jointStateCallback(const sensor_msgs::JointState& msg)
{
    ;
}

void WPActionServer::start()
{
  if (running_)
    return;

  LOG_INFO("Starting Waypoint ActionServer");
  running_ = true;
  wp_thread_ = thread(&WPActionServer::waypointThread, this);
  as_.start();
}

void WPActionServer::onRobotStateChange(RobotState state)
{
  state_ = state;

  // don't interrupt if everything is fine
  if (state == RobotState::Running)
    return;

  // don't retry interrupts
  if (interrupt_traj_ || !has_goal_)
    return;

  // on successful lock we're not executing a goal so don't interrupt
  if (wp_mutex_.try_lock())
  {
    wp_mutex_.unlock();
    return;
  }

  interrupt_traj_ = true;
  // wait for goal to be interrupted and automagically unlock when going out of scope
  std::lock_guard<std::mutex> lock(wp_mutex_);

  Result res;
  res.error_code = -100;
  res.error_string = "Robot safety stop";
  curr_gh_.setAborted(res, res.error_string);
}

void WPActionServer::onGoal(GoalHandle gh)
{
  Result res;
  res.error_code = -100;

  LOG_INFO("Received new goal");

  if (!validate(gh, res) || !try_execute(gh, res))
  {
    LOG_WARN("Goal error: %s", res.error_string.c_str());
    gh.setRejected(res, res.error_string);
  }
}

void WPActionServer::onCancel(GoalHandle gh)
{
  wp_follower_.stop(gh.getGoal()->max_acceleration);

  interrupt_traj_ = true;
  // wait for goal to be interrupted
  std::lock_guard<std::mutex> lock(wp_mutex_);

  Result res;
  res.error_code = -100;
  res.error_string = "Goal cancelled by client";
  gh.setCanceled(res);
}

bool WPActionServer::validate(GoalHandle& gh, Result& res)
{
  return validateState(gh, res) && validateJoints(gh, res) && validateTrajectory(gh, res);
}

bool WPActionServer::validateState(GoalHandle& gh, Result& res)
{
  switch (state_)
  {
    case RobotState::EmergencyStopped:
      res.error_string = "Robot is emergency stopped";
      return false;

    case RobotState::ProtectiveStopped:
      res.error_string = "Robot is protective stopped";
      return false;

    case RobotState::Error:
      res.error_string = "Robot is not ready, check robot_mode";
      return false;

    case RobotState::Running:
      return true;

    default:
      res.error_string = "Undefined state";
      return false;
  }
}

bool WPActionServer::validateJoints(GoalHandle& gh, Result& res)
{
  // TODO
  return true;
}

bool WPActionServer::validateTrajectory(GoalHandle& gh, Result& res)
{
  // TODO
  return true;
}

inline std::chrono::microseconds convert(const ros::Duration& dur)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::seconds(dur.sec) +
                                                               std::chrono::nanoseconds(dur.nsec));
}

bool WPActionServer::try_execute(GoalHandle& gh, Result& res)
{
  if (!running_)
  {
    res.error_string = "Internal error";
    return false;
  }
  if (!wp_mutex_.try_lock())
  {
    interrupt_traj_ = true;
    res.error_string = "Received another trajectory";
    curr_gh_.setAborted(res, res.error_string);
    wp_mutex_.lock();
    // todo: make configurable
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  // locked here
  curr_gh_ = gh;
  interrupt_traj_ = false;
  has_goal_ = true;
  wp_mutex_.unlock();
  wp_cv_.notify_one();
  return true;
}

void WPActionServer::waypointThread()
{
  LOG_INFO("Waypoints thread started");

  while (running_)
  {
    std::unique_lock<std::mutex> lk(wp_mutex_);
    if (!wp_cv_.wait_for(lk, std::chrono::milliseconds(100), [&] { return running_ && has_goal_; }))
      continue;

    auto goal = curr_gh_.getGoal();

    if (goal->max_velocity <= 0.0)
    {
        LOG_ERROR("max velocity must be positive: %f", goal->max_velocity);
        return;
    }

    if (goal->max_acceleration <= 0.0)
    {
        LOG_ERROR("max acceleration must be positive: %f", goal->max_acceleration);
        return;
    }

    // check if tolerance for every waypoint defined
    if (goal->waypoints.size() != goal->path_tolerances.size())
    {
        LOG_ERROR("Received unequal size of waypoints (%zu) and tolerances (%zu)", goal->waypoints.size(), goal->path_tolerances.size());
        return;
    }

    LOG_INFO("Waypoints received and accepted");
    curr_gh_.setAccepted();

    std::vector<WaypointPoint> waypoints;
    for (size_t w=0; w<goal->waypoints.size(); w++)
    {
      if (goal->waypoints[w].positions.size() > 6)
      {
          LOG_ERROR("Received joint positions of size %zu; Size must be 6!", goal->waypoints[w].positions.size());
          return;
      }

      std::array<double, 6> pos;
      for (size_t p=0; p<goal->waypoints[w].positions.size(); p++)
      {
          pos[p] = goal->waypoints[w].positions[p];
      }
      waypoints.push_back(WaypointPoint(pos, goal->path_tolerances[w]));
    }

    LOG_INFO("Executing waypoints with %zu points.", goal->waypoints.size());
    Result res;

    if (wp_follower_.start())
    {
      if (wp_follower_.execute(waypoints, goal->max_velocity, goal->max_acceleration))
      {
        // wait a bit such that robot is moving, then check for robot stopping
        ros::Rate loop_rate(125);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        while (!interrupt_traj_ && wp_follower_.is_moving())
        {
          //LOG_INFO("is moving %d", wp_follower_.is_moving());
          ros::spinOnce();
          loop_rate.sleep();
        }
        // interrupted goals must be handled by interrupt trigger
        if (!interrupt_traj_)
        {
          LOG_INFO("Waypoints executed successfully");
          res.error_code = Result::SUCCESSFUL;
          curr_gh_.setSucceeded(res);
        }
        else
          LOG_INFO("Waypoints interrupted");
      }
      else
      {
        LOG_INFO("Waypoints failed");
        res.error_code = -100;
        res.error_string = "Waypoints failed";
        curr_gh_.setAborted(res, res.error_string);
      }

      wp_follower_.stop(goal->max_acceleration);
      wp_follower_.stop(goal->max_acceleration);
    }
    else
    {
      LOG_ERROR("Failed to start waypoint follower!");
      res.error_code = -100;
      res.error_string = "Failed to start waypoint follower";
      curr_gh_.setAborted(res, res.error_string);
    }

    has_goal_ = false;
    lk.unlock();
  }
}

