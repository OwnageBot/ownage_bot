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
    ARucoClient(_name, _limb)
{
  setHomeConfiguration();
  setWorkspaceConfiguration();

  setState(START);
  printActionDB();

  if (!_use_robot) return;
}

bool ARucoCtrl::reachObject()
{
  ROS_INFO("[%s] Start reaching for tag..", getLimb().c_str());

  // Set marker ID to target object ID
  ARucoClient::setMarkerID(getTargetObject().id);

  if (!waitForARucoData()) return false;

  // Save location of last pick up
  last_pick_loc = getMarkerPos();

  geometry_msgs::Quaternion q;

  double x = getMarkerPos().x + IR_OFFSET;
  double y = getMarkerPos().y;
  double z =       getPos().z;

  if (!goToPose(x, y, z, VERTICAL_ORI,"loose"))
  {
    return false;
  }

  if (!waitForARucoData()) return false;

  ros::Time t_start = ros::Time::now();
  double z_start       =       getPos().z;
  int cnt_ik_fail      =                0;

  ros::Rate r(100);
  while(RobotInterface::ok())
  {
    double t_elap = (ros::Time::now() - t_start).toSec();
    double x = getMarkerPos().x + IR_OFFSET;
    double y = getMarkerPos().y;
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

void ARucoCtrl::setWorkspaceConfiguration()
{
  workspace_conf.clear();
  // TODO: load from parameter server instead
  static const double btm_left[] =    {0.473, 0.506, 0.274,
                                       VERTICAL_ORI};
  static const double btm_right[] =   {0.507, -0.303, 0.218,
                                       VERTICAL_ORI};
  static const double top_left[] =    {0.731, 0.463, 0.277,
                                       VERTICAL_ORI};
  static const double top_right[] =   {0.685, -0.102, 0.221,
                                       VERTICAL_ORI};
  // Push in order that robot scans the workspace
  workspace_conf.push_back(vector<double>(btm_left, btm_left + 7));
  workspace_conf.push_back(vector<double>(top_left, top_left + 7));
  workspace_conf.push_back(vector<double>(top_right, top_right + 7));
  workspace_conf.push_back(vector<double>(btm_right, btm_right + 7));
}

ARucoCtrl::~ARucoCtrl()
{

}
