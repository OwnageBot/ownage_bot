#include "object_picker.h"

using namespace std;
using namespace baxter_core_msgs;

ObjectPicker::ObjectPicker(
    std::string _name,
    std::string _limb,
    bool _no_robot ) :
    ArmCtrl(_name,_limb, _no_robot),
    ARucoClient(_name, _limb),
    elap_time(0)
{
  setHomeConfiguration();

  setWorkspaceConfiguration();
  
  setState(START);

  insertAction(ACTION_GET,
               static_cast<f_action>(&ObjectPicker::pickObject));
  insertAction(ACTION_SCAN,
               static_cast<f_action>(&ObjectPicker::scanWorkspace));

  printActionDB();

  _new_obj_sub = _n.subscribe("/object_tracker/new_object",
                              SUBSCRIBER_BUFFER,
                              &ObjectPicker::newObjectCallback, this);

  if (_no_robot) return;

  if (!callAction(ACTION_HOME)) setState(ERROR);
}

void ObjectPicker::newObjectCallback(const std_msgs::UInt32 msg)
{
  if (!isObjectInDB(msg.data)) {
    stringstream object_name;
    object_name << "object" << msg.data;
    insertObject(msg.data, object_name.str());
    ROS_INFO("[%s] ID %d added to database", getLimb().c_str(), msg.data);
  }
}

bool ObjectPicker::pickObject()
{
  if (!homePoseStrict())          return false;
  ros::Duration(0.05).sleep();
  if (!pickARTag())               return false;
  if (!gripObject())              return false;
  if (!moveArm("up", 0.3))        return false;
  if (!hoverAboveTable(Z_LOW))    return false;

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
  }

  return true;
}

void ObjectPicker::recoverFromError()
{
  if (getInternalRecovery() == true)
  {
    // Release object and go home
    hoverAboveTable(Z_LOW);
    releaseObject();
    goHome();
  }
}

bool ObjectPicker::pickARTag()
{
  ROS_INFO("[%s] Start Picking up tag..", getLimb().c_str());

  if (!is_ir_ok())
  {
    ROS_ERROR("No callback from the IR sensor! Stopping.");
    return false;
  }

  if (!waitForARucoData()) return false;

  geometry_msgs::Quaternion q;

  double x = getMarkerPos().x;
  double y = getMarkerPos().y + 0.04;
  double z =       getPos().z;

  ROS_DEBUG("Going to: %g %g %g", x, y, z);
  if (!goToPose(x, y, z, POOL_ORI_L,"loose"))
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

    double x = getMarkerPos().x;
    double y = getMarkerPos().y;
    double z = z_start - ARM_SPEED * new_elap_time;

    ROS_DEBUG("Time %g Going to: %g %g %g", new_elap_time, x, y, z);

    if (goToPoseNoCheck(x,y,z,POOL_ORI_L))
    {
      cnt_ik_fail = 0;
      if (new_elap_time - elap_time > 0.02)
      {
        ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
      }
      elap_time = new_elap_time;

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


void ObjectPicker::setHomeConfiguration()
{
  // Home location at center of the table
  setHomeConf(0.1967, -0.8702, -1.0531,  1.5578,
              0.6516,  1.2464, -0.1787);
}

void ObjectPicker::setWorkspaceConfiguration()
{
  workspace_conf.clear();
  // Hard-coded workspace boundaries, needs to be updated
  static const double btm_left[] =    {0.473, 0.506, 0.274,
                                       0, 1, 0, 0};
  static const double btm_right[] =   {0.507, -0.303, 0.218,
                                       0, 1, 0, 0};
  static const double top_left[] =    {0.731, 0.463, 0.277,
                                       0, 1, 0, 0};
  static const double top_right[] =   {0.685, -0.102, 0.221,
                                       0, 1, 0, 0};
  // Push in order that robot scans the workspace
  workspace_conf.push_back(std::vector<double>(btm_left, btm_left + 7));
  workspace_conf.push_back(std::vector<double>(top_left, top_left + 7));
  workspace_conf.push_back(std::vector<double>(top_right, top_right + 7));
  workspace_conf.push_back(std::vector<double>(btm_right, btm_right + 7));
}

void ObjectPicker::setObjectID(int _obj)
{
    ArmCtrl::setObjectID(_obj);
    ARucoClient::setMarkerID(_obj);
}

ObjectPicker::~ObjectPicker()
{

}
