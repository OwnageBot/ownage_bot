#include "object_picker.h"

using namespace std;
using namespace baxter_core_msgs;
using namespace human_robot_collaboration_msgs;
using namespace ownage_bot;

ObjectPicker::ObjectPicker(
    string _name,
    string _limb,
    bool _use_robot ) :
    ArmCtrl(_name,_limb, _use_robot),
    ARucoClient(_name, _limb),
    elap_time(0)
{
  setHomeConfiguration();

  setWorkspaceConfiguration();

  setState(START);

  insertAction(ACTION_SCAN,
               static_cast<f_action>(&ObjectPicker::scanWorkspace));
  insertAction(ACTION_FIND,
               static_cast<f_action>(&ObjectPicker::findObject));
  insertAction(ACTION_GET,
               static_cast<f_action>(&ObjectPicker::pickObject));
  insertAction(ACTION_PUT,
               static_cast<f_action>(&ObjectPicker::putObject));
  insertAction(ACTION_OFFER,
               static_cast<f_action>(&ObjectPicker::offerObject));
  insertAction(ACTION_REPLACE,
               static_cast<f_action>(&ObjectPicker::replaceObject));
  insertAction(ACTION_WAIT,
               static_cast<f_action>(&ObjectPicker::waitForFeedback));

  printActionDB();

  loc_obj_client =
    nh.serviceClient<LocateObject>("/ownage_bot/locate_object");
    
  if (!_use_robot) return;
}

bool ObjectPicker::findObject()
{
  // Request last-remembered location of object from ObjectTracker node
  LocateObject srv;
  srv.request.id = getTargetObject().id;
  if (!loc_obj_client.call(srv)) {
    ROS_ERROR("[%s] Failed to call service locate_object!", getLimb().c_str());
    return false;
  }
  geometry_msgs::Point p = srv.response.pose.position;
  ROS_DEBUG("Finding object at x=%g, y=%g...", p.x, p.y);
  // if (!homePoseStrict()) return false;
  ros::Duration(0.05).sleep();
  // Hover above last-remembered location
  if (!goToPose(p.x, p.y, Z_FIND, VERTICAL_ORI)) {
    ROS_ERROR("[%s] Failed to go to object location!\n", getLimb().c_str());
    return false;
  }
  if (!loc_obj_client.call(srv)) {
    ROS_ERROR("[%s] Failed to call service locate_object!\n", getLimb().c_str());
    return false;
  }
  p = srv.response.pose.position;
  // Hover above new location
  if (!goToPose(p.x, p.y, Z_FIND, VERTICAL_ORI)) return false;

  return true;
}

bool ObjectPicker::offerObject()
{
 // Request last-remembered location of object from ObjectTracker node
  LocateObject srv;
  srv.request.id = getTargetObject().id;
  if (!loc_obj_client.call(srv)) {
    ROS_ERROR("[%s] Failed to call service locate_object!", getLimb().c_str());
    return false;
  }
  if (!srv.response.success) {
    ROS_ERROR("[%s] Object %d not tracked!\n", getLimb().c_str(), srv.request.id);
    return false;
  }
  geometry_msgs::Point p = srv.response.pose.position;
  // if (!homePoseStrict()) return false;
  ros::Duration(0.05).sleep();
  // Hover above last-remembered location and offer obj
  if (!goToPose(p.x, p.y, Z_LOW, VERTICAL_ORI)) {
    ROS_ERROR("[%s] Failed to go to object location!\n", getLimb().c_str());
    return false;
  }
  // Slowly rotate arm to face human
  sensor_msgs::JointState j = getJointStates();
  j.position[5] = -0.5;
  ros::Rate r(100);
  while(RobotInterface::ok() && !isConfigurationReached(j.position))
  {
    goToJointConfNoCheck(j.position);
    r.sleep();
  }
  // Sleep and wait for user input
  ros::Duration(3).sleep();
  return true;
}

bool ObjectPicker::pickObject()
{
  // Check if object is currently held
  if (Gripper::is_gripping()) {
    setSubState(OBJECT_HELD);
    return false;
  }
  if (!pickARTag())               return false;
  if (!Gripper::close())              return false;
  // Move up from current position to Z_LOW
  geometry_msgs::Point p = getPos();
  if (!goToPose(p.x, p.y, Z_LOW, VERTICAL_ORI)) return false;
  return true;
}

bool ObjectPicker::putObject()
{
  // Check if object is currently held
  if (!Gripper::is_gripping()) {
    setSubState(NO_OBJECT_HELD);
    return false;
  }
  // Move down from current position to Z_RELEASE
  geometry_msgs::Point p = getPos();
  ros::Duration(0.05).sleep();
  if (!goToPose(p.x, p.y, Z_RELEASE, VERTICAL_ORI)) return false;
  ros::Duration(1).sleep();
  Gripper::open();

  return true;
}

bool ObjectPicker::replaceObject()
{
  // Check if object is currently held
  if (!Gripper::is_gripping()) {
    setSubState(NO_OBJECT_HELD);
    return false;
  }
  // Move to location of last picked object and release
  if (!goToPose(last_pick_loc.x, last_pick_loc.y,
                Z_RELEASE, VERTICAL_ORI)) return false;
  ros::Duration(1).sleep();
  Gripper::open();

  return true;
}

bool ObjectPicker::scanWorkspace()
{
  ROS_INFO("[%s] Scanning workspace...", getLimb().c_str());

  for(int i = 0; i < workspace_conf.size(); i++) {
    ROS_INFO("[%s] Going to corner %d", getLimb().c_str(), i);
    int r = ArmCtrl::goToPose(
      workspace_conf[i][0], workspace_conf[i][1], workspace_conf[i][2],
      workspace_conf[i][3], workspace_conf[i][4],
      workspace_conf[i][5], workspace_conf[i][6]);
    if (!r) ROS_ERROR("Could not reach corner %d, continuing", i);
    ros::Duration(0.25).sleep();
  }

  return true;
}

bool ObjectPicker::waitForFeedback()
{
  printf("[%s] Waiting for feedback...\n", getLimb().c_str());
  setState(WORKING);
  ros::Duration(3).sleep();
  return true;
}

bool ObjectPicker::goHome()
{
    if (!homePoseStrict()) return true;
    if (!goToPose(home_loc.x, home_loc.y, home_loc.z, VERTICAL_ORI)) return false;
    return true;
}

void ObjectPicker::recoverFromError()
{
  ROS_INFO("[%s] Recovering from error", getLimb().c_str());
}

bool ObjectPicker::pickARTag()
{
  ROS_INFO("[%s] Start Picking up tag..", getLimb().c_str());

  // Set marker ID to target object ID
  ARucoClient::setMarkerID(tgt_object.id);

  if (!waitForARucoData()) return false;

  // Save location of last pick up
  last_pick_loc = getMarkerPos();

  geometry_msgs::Quaternion q;

  double x = getMarkerPos().x + IR_OFFSET;
  double y = getMarkerPos().y;
  double z =       getPos().z;

  printf("Going to: %g %g %g", x, y, z);
  if (!goToPose(x, y, z, VERTICAL_ORI,"loose"))
  {
    return false;
  }

  if (!waitForARucoData()) return false;

  ros::Time start_time = ros::Time::now();
  double z_start       =       getPos().z;
  int cnt_ik_fail      =                0;

  ros::Rate r(100);
  while(RobotInterface::ok())
  {
    double new_elap_time = (ros::Time::now() - start_time).toSec();

    double x = getMarkerPos().x + IR_OFFSET;
    double y = getMarkerPos().y;
    double z = z_start - 0.8 * ARM_SPEED * new_elap_time;

    ROS_DEBUG("Time %g Going to: %g %g %g", new_elap_time, x, y, z);

    if (goToPoseNoCheck(x, y, z, VERTICAL_ORI))
    {
      cnt_ik_fail = 0;
      if (new_elap_time - elap_time > 0.02)
      {
        ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
      }
      elap_time = new_elap_time;

      // Use loose collision detection to avoid over-pushing
      if(hasCollidedIR("strict"))
      {
        ROS_DEBUG("Collision!");
        setSubState(ACTION_GET);
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

bool ObjectPicker::releaseAtPose(double px, double py, double pz,
                                 double ox, double oy, double oz, double ow,
                                 string mode)
{
  ros::Time start_time = ros::Time::now();
  double z_start       =       getPos().z;
  int cnt_ik_fail      =                0;

  ros::Rate r(100);
  while(RobotInterface::ok())
  {
    double new_elap_time = (ros::Time::now() - start_time).toSec();

    // Move to release point bit by bit, make sure z does not exceed
    double x = px;
    double y = py;
    double z = (z < pz) ? pz : z_start - ARM_SPEED * new_elap_time;

    ROS_DEBUG("Time %g Going to: %g %g %g", new_elap_time, px, py, z);

    if (goToPoseNoCheck(x,y,z,VERTICAL_ORI))
    {
      cnt_ik_fail = 0;
      if (new_elap_time - elap_time > 0.02)
      {
        ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
      }
      elap_time = new_elap_time;

      // Release object upon reaching pose, or collision
      if(z == pz ||
         isPoseReached(px, py, pz, ox, oy, oz, ow, mode) ||
         getWrench().force.z < RELEASE_THRESHOLD)
      {
        ros::Duration(1).sleep();
        Gripper::open();
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
}

void ObjectPicker::setHomeConfiguration()
{
  // Home location at center of the table
  setHomeConf(0.109, -0.930, -1.507, 2.070,
              0.735, 1.150, -0.956);
  home_loc.x = 0.50;  home_loc.y = 0.176;  home_loc.z = Z_LOW;
}

void ObjectPicker::setWorkspaceConfiguration()
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

ObjectPicker::~ObjectPicker()
{

}
