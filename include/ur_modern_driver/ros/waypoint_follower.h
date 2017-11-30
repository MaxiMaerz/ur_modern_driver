#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <ur_msgs/IOStates.h>
#include <vector>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/commander.h"

struct WaypointPoint
{
  std::array<double, 6> positions;
  double tolerance;

  WaypointPoint() {}
  WaypointPoint(std::array<double, 6> &pos, double tol) 
      : positions(pos)
      , tolerance(tol)
    {}
};

class WaypointFollower
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  std::atomic<bool> running_;
  bool robot_moving_;
  URCommander &commander_;
  std::string program_;

  void ioCallback(const ur_msgs::IOStates::ConstPtr& msg);

public:
  WaypointFollower(URCommander &commander);

  bool start();
  bool execute(std::vector<WaypointPoint> &waypoints, float max_velocity, float max_acceleration);
  bool is_moving();
  bool uploadProgram();
  void stop(float max_acceleration);
};

