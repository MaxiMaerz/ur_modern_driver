#include "ur_modern_driver/ros/waypoint_follower.h"
#include <endian.h>
#include <cmath>
#include <ros/ros.h>

static const std::string PROGRAM_BODY_REPLACE("{{PROGRAM_BODY}}");
static const std::string WAYPOINT_PROGRAM = R"(
def driverProg():
  {{PROGRAM_BODY}}
end
)";
static const std::string MOVEL_PROGRAM = R"(movel([{{J0}}, {{J1}}, {{J2}}, {{J3}}, {{J4}}, {{J5}}], a={{ACC}}, v={{VEL}}, r={{TOL}})
)";

WaypointFollower::WaypointFollower(URCommander &commander)
  : running_(false)
  , commander_(commander)
{
  std::string res(WAYPOINT_PROGRAM);
  std::string empty("\n");
  res.replace(res.find(PROGRAM_BODY_REPLACE), PROGRAM_BODY_REPLACE.length(), empty);
  program_ = res;
}

bool WaypointFollower::start()
{
  return (running_ = true);
}

bool WaypointFollower::execute(std::vector<WaypointPoint> &waypoints, float max_velocity, float max_acceleration)
{
  if (!running_)
    return false;

  std::string program("");
  
  for (const auto& point : waypoints)
  {
    LOG_INFO("waypointfollower point: ([%f,%f,%f,%f,%f,%f])", point.positions[0], point.positions[1], point.positions[2], point.positions[3], point.positions[4], point.positions[5]);
    LOG_INFO("tolerance: %f", point.tolerance);

    std::string movel(MOVEL_PROGRAM);
    for (size_t p=0; p<point.positions.size(); p++)
    {
        std::string joint = std::string("{{J")+std::to_string(p)+std::string("}}");
        movel.replace(movel.find(joint), joint.length(), std::to_string(point.positions[p]));
    }
    movel.replace(movel.find("{{VEL}}"), 7, std::to_string(max_velocity));
    movel.replace(movel.find("{{ACC}}"), 7, std::to_string(max_acceleration));
    movel.replace(movel.find("{{TOL}}"), 7, std::to_string(point.tolerance));
    program.append(movel);
  }
  std::string res(WAYPOINT_PROGRAM);
  res.replace(res.find(PROGRAM_BODY_REPLACE), PROGRAM_BODY_REPLACE.length(), program);
  // TODO why only second program is executed??
  stop();
  program_ = res;
  return uploadProgram();
}

void WaypointFollower::stop()
{
  if (!running_)
    return;

  LOG_INFO("Writing waypoint stop program");
  std::string res(WAYPOINT_PROGRAM);
  // TODO stopl with max acceleration
  // make stop string std::string("stopl()")
  //res.replace(res.find(PROGRAM_BODY_REPLACE), PROGRAM_BODY_REPLACE.length(), stop string);
  std::string stop_popup("popup(\"stop\", title=\"Stop Popup\")");
  res.replace(res.find(PROGRAM_BODY_REPLACE), PROGRAM_BODY_REPLACE.length(), stop_popup);
  program_ = res;
  uploadProgram();

  running_ = false;
}

bool WaypointFollower::uploadProgram()
{
  LOG_INFO("Uploading waypoint program to robot");
  if (!commander_.uploadProg(program_))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }
  std::cout << program_ << std::endl;
  LOG_INFO("Uploaded waypoint program to robot");
  return true;
}
