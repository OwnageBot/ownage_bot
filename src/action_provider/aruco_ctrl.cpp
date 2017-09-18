#include "aruco_ctrl.h"

using namespace std;
using namespace baxter_core_msgs;
using namespace human_robot_collaboration_msgs;
using namespace ownage_bot;

ARucoCtrl::ARucoCtrl(
    string _name,
    string _limb,
    bool _use_robot ) :
    BaxterArmCtrl(_name,_limb, _use_robot),
    PerceptionClientImpl(_name, _limb)
{
  setHomeConfiguration();

  setState(START);
  printActionDB();

  if (!_use_robot) return;
}

bool ARucoCtrl::reachObject()
{
  ROS_INFO("[%s] Start reaching for tag..", getLimb().c_str());

  // Set marker ID to target object ID
  PerceptionClientImpl::setObjectID(getTargetObject().id);

  if (!PerceptionClientImpl::waitForData()) return false;

  // Save location of last pick up
  last_pick_loc = curr_object_pos;

  geometry_msgs::Quaternion q;

  double x = curr_object_pos.x + IR_OFFSET;
  double y = curr_object_pos.y;
  double z = curr_object_pos.z;

  if (!goToPose(x, y, z, VERTICAL_ORI,"loose"))
  {
    return false;
  }

  if (!PerceptionClientImpl::waitForData()) return false;

  ros::Time t_start = ros::Time::now();
  double z_start       =       getPos().z;
  int cnt_ik_fail      =                0;

  ros::Rate r(100);
  while(RobotInterface::ok())
  {
    double t_elap = (ros::Time::now() - t_start).toSec();
    double x = curr_object_pos.x + IR_OFFSET;
    double y = curr_object_pos.y;
    double z = z_start - 0.8 * ARM_SPEED * t_elap;

    if (goToPoseNoCheck(x, y, z, VERTICAL_ORI))
    {
      cnt_ik_fail = 0;

      // Check IR and force sensors for collision
      if(hasCollidedIR("strict") || getWrench().force.z < REACH_THRESHOLD)
      {
        ROS_DEBUG("Collision!");
        return true;
      }

      r.sleep();
    }
    else
    {
      cnt_ik_fail++;
    }

    if (cnt_ik_fail == 10)
    {
      return false;
    }
  }

  return false;
}

void ARucoCtrl::setHomeConfiguration()
{
  // Home location at center of the table
  setHomeConf(0.109, -0.930, -1.507, 2.070,
              0.735, 1.150, -0.956);
  home_loc.x = 0.50;  home_loc.y = 0.176;  home_loc.z = Z_LOW;
}

ARucoCtrl::~ARucoCtrl()
{

}
