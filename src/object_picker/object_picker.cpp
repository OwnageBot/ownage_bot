#include "object_picker.h"

using namespace std;
using namespace baxter_core_msgs;
using namespace baxter_collaboration_msgs;
using namespace ownage_bot;

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

  insertAction(ACTION_FIND,
               static_cast<f_action>(&ObjectPicker::findObject));
  insertAction(ACTION_GET,
               static_cast<f_action>(&ObjectPicker::pickObject));
  insertAction(ACTION_PUT,
               static_cast<f_action>(&ObjectPicker::putObject));
  insertAction(ACTION_SCAN,
               static_cast<f_action>(&ObjectPicker::scanWorkspace));

  printActionDB();

  _new_obj_sub = _n.subscribe("/object_tracker/new_object",
                              SUBSCRIBER_BUFFER,
                              &ObjectPicker::newObjectCallback, this);

  _loc_obj_client =
    _n.serviceClient<LocateObject>("/object_tracker/locate_object");

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

bool ObjectPicker::findObject()
{
  // Request last-remembered location of object from ObjectTracker node
  LocateObject srv;
  srv.request.id = getObjectID();
  if (!_loc_obj_client.call(srv)) {
    ROS_ERROR("[%s] Failed to call service locate_object!", getLimb().c_str());
    return false;
  }
  geometry_msgs::Point p = srv.response.pose.position;
  ros::Duration(0.05).sleep();
  // Hover above last-remembered location
  if (!goToPose(p.x, p.y, Z_LOW, VERTICAL_ORI_L)) return false;
  // Check if object is indeed there, return false otherwise
  if (!waitForARucoData()) return false;
  // Request again just in case object shifted slightly
  if (!_loc_obj_client.call(srv)) {
    ROS_ERROR("[%s] Failed to call service locate_object!", getLimb().c_str());
    return false;
  }
  p = srv.response.pose.position;
  // Hover above new location
  if (!goToPose(p.x, p.y, Z_LOW, VERTICAL_ORI_L)) return false;

  return true;
}

bool ObjectPicker::pickObject()
{
  if (!pickARTag())               return false;
  if (!gripObject())              return false;
  // Move up from current position to Z_LOW
  geometry_msgs::Point p = getPos();
  if (!goToPose(p.x, p.y, Z_LOW, VERTICAL_ORI_L)) return false;

  return true;
}

bool ObjectPicker::putObject()
{
  // Move down from current position to Z_RELEASE
  geometry_msgs::Point p = getPos();
  if (!goToPose(p.x, p.y, Z_RELEASE, VERTICAL_ORI_L)) return false;
  ros::Duration(0.5).sleep();
  releaseObject();

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

void ObjectPicker::recoverFromError()
{
  if (getInternalRecovery() == true)
  {
    // Release object and go home
    hoverAboveTable(Z_RELEASE);
    releaseObject();
    homePoseStrict();
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

bool ObjectPicker::serviceCb(DoAction::Request  &req, DoAction::Response &res)
{
    // Let's read the requested action and object to act upon
    setSubState("");
    //object_ids.clear();
    setObjectID(-1);

    string action = req.action;
    std::vector<int> object_ids;
    std::string objs_str = "";

    for (size_t i = 0; i < req.objects.size(); ++i)
    {
        object_ids.push_back(req.objects[i]);
        objs_str += toString(req.objects[i]) + ", ";
    }
    objs_str = objs_str.substr(0, objs_str.size()-2); // Remove the last ", "

    printf("[%s] Service request received. Action: %s Objects: %s",
             getLimb().c_str(), action.c_str(), objs_str.c_str());

    // Print the action or object DB if requested by the user
    if (action == LIST_ACTIONS)
    {
        printActionDB();
        res.success  = true;
        res.response = actionDBToString();
        return true;
    }
    else if (action == LIST_OBJECTS)
    {
        printObjectDB();
        res.success  = true;
        res.response = objectDBToString();
        return true;
    }

    res.success = false;

    setAction(action);

    // Only pickObject needs to know the object id
    if (action == ACTION_GET || action == ACTION_FIND)
    {
        setObjectIDs(areObjectsInDB(object_ids));

        if (object_ids.size() == 0)
        {
            res.response = OBJ_NOT_IN_DB;
            ROS_ERROR("[%s] Requested object(s) are not in the database!",
                                                       getLimb().c_str());
            return true;
        }
        else if (object_ids.size() == 1)
        {
            setObjectID(object_ids[0]);
        }
        else if (object_ids.size() >  1)
        {
            // Defaults to first object in list, ignores others
            setObjectID(chooseObjectID(object_ids));
        }
    }

    startInternalThread();

    // This is there for the current thread to avoid overlapping
    // with the internal thread that just started
    ros::Duration(0.5).sleep();

    ros::Rate r(THREAD_FREQ);
    while( ros::ok() && ( int(getState()) != START   &&
                          int(getState()) != ERROR   &&
                          int(getState()) != DONE      ))
    {
        if (ros::isShuttingDown())
        {
            setState(KILLED);
            return true;
        }

        if (getState() == KILLED)
        {
            // Send ACT_CANCELLED if cuff button is pressed
            res.response = ACT_CANCELLED;
            recoverFromError();
        }

        r.sleep();
    }

    if ( int(getState()) == START   ||
         int(getState()) == DONE      )
    {
        res.success = true;
    }

    if (getState() == ERROR)
    {
        res.response = getSubState();
    }

    ROS_INFO("[%s] Service reply with success: %s\n",
             getLimb().c_str(), res.success?"true":"false");
    return true;
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
  // TODO: load from parameter server instead
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
