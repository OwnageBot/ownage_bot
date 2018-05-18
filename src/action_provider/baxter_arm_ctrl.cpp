#include "baxter_arm_ctrl.h"

using namespace std;
using namespace geometry_msgs;
using namespace baxter_core_msgs;
using namespace ownage_bot;

BaxterArmCtrl::BaxterArmCtrl(string _name, string _limb,
                             bool _use_robot, bool _use_forces,
                             bool _use_trac_ik, bool _use_cart_ctrl) :
                RobotInterface(_name,_limb, _use_robot, THREAD_FREQ,
                               _use_forces, _use_trac_ik, _use_cart_ctrl),
                Gripper(_limb, _use_robot),
                sub_state(""), action(""), prev_action(""),
                internal_recovery(false), home_conf(7), arm_speed(ARM_SPEED),
		tgt_object(ObjectMsg()), tgt_location(Point())
{
    std::string other_limb = getLimb() == "right" ? "left" : "right";

    std::string topic = "/"+getName()+"/service_"+_limb;
    service = nh.advertiseService(topic, &BaxterArmCtrl::serviceCb, this);
    ROS_INFO("[%s] Created service server with name  : %s",
             getLimb().c_str(), topic.c_str());
    string cancel = "/"+getName()+"/cancel_"+_limb;
    cancel_srv = nh.advertiseService(cancel, &BaxterArmCtrl::cancelCb, this);

    lookup_obj_client =
        nh.serviceClient<LookupObject>("/ownage_bot/lookup_object");

    insertAction(ACT_REQ::_ACTION_GOHOME,
		 &BaxterArmCtrl::goHome, TARGET_NONE);
    insertAction(ACT_REQ::_ACTION_RELEASE,
		 &BaxterArmCtrl::releaseObject, TARGET_NONE);
    insertAction(ACT_REQ::_ACTION_MOVETO,
		 &BaxterArmCtrl::moveToLocation, TARGET_LOCATION);
    insertAction(ACT_REQ::_ACTION_FIND,
		 &BaxterArmCtrl::findObject, TARGET_OBJECT);
    insertAction(ACT_REQ::_ACTION_PICKUP,
		 &BaxterArmCtrl::pickObject, TARGET_OBJECT);
    insertAction(ACT_REQ::_ACTION_PUTDOWN,
		 &BaxterArmCtrl::putObject, TARGET_NONE);
    insertAction(ACT_REQ::_ACTION_OFFER,
		 &BaxterArmCtrl::offerObject, TARGET_OBJECT);
    insertAction(ACT_REQ::_ACTION_REPLACE,
		 &BaxterArmCtrl::replaceObject, TARGET_NONE);
    insertAction(ACT_REQ::_ACTION_WAIT,
		 &BaxterArmCtrl::waitForFeedback, TARGET_NONE);
}

bool BaxterArmCtrl::startThread()
{
    // Need to join the old thread in order to spin out a new one.
    if (arm_thread.joinable())    { arm_thread.join(); };

    arm_thread = std::thread(&BaxterArmCtrl::InternalThreadEntry, this);
    return arm_thread.joinable();
}

void BaxterArmCtrl::InternalThreadEntry()
{
    ROS_INFO("[%s] Internal_recovery flag set to %s", getLimb().c_str(),
                                internal_recovery==true?"true":"false");

    std::string a =     getAction();
    int         s = int(getState());

    setState(WORKING);

    if (not isRobotUsed())
    {
        ros::Duration(2.0).sleep();
        setState(DONE);
    }
    else if (a == ACT_REQ::_ACTION_GOHOME || a == ACT_REQ::_ACTION_RELEASE)
    {
        if (callAction(a))   setState(DONE);
    }
    else if (s == START || s == ERROR || s == DONE  || s == KILLED )
    {
        if (callAction(a))   setState(DONE);
        else                 setState(ERROR);
    }
    else
    {
        ROS_ERROR("[%s] Invalid Action %s in state %i",
                  getLimb().c_str(), a.c_str(), s);
    }

    if (int(getState())==WORKING)
    {
        setState(ERROR);
    }

    if (int(getState())==ERROR)
    {
        ROS_ERROR("[%s] Action %s not successful! State %s %s",
                  getLimb().c_str(), a.c_str(), string(getState()).c_str(),
                  getSubState().c_str());

        if (internal_recovery)
        {
            recoverFromError();
        }
    }

    return;
}

bool BaxterArmCtrl::serviceCb(CallAction::Request &req,
                              CallAction::Response &res)
{
    // Let's read the requested action and object to act upon
    setSubState("");

    string action = req.action;

    ROS_INFO("[%s] Service request received. Action: %s",
             getLimb().c_str(), action.c_str());

    // Print the action or object DB if requested by the user
    if (action == LIST_ACTIONS)
    {
        printActionDB();
        res.success = true;
        res.response = actionDBToString();
        return true;
    }

    if (!isActionInDB(action)) // The action is in the db
    {
        ROS_ERROR("[%s] Action %s is not in the database!",
                  getLimb().c_str(), action.c_str());
        res.success = false;
        res.response = ACT_RESP::_ACT_NOT_IN_DB;
        return true;
    }

    string target = action_db[action].target;

    if (target == TARGET_OBJECT)
    {
        setTargetObject(req.object);
    }
    else if (target == TARGET_LOCATION)
    {
      setTargetLocation(req.location);
    }

    setAction(action);
    res.success = false;

    startThread();

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
	  res.response = ACT_RESP::_ACT_KILLED;
	  break;
        }

      r.sleep();
    }

    if (int(getState()) == START || int(getState()) == DONE)
    {
      res.success = true;
    }

    if (getState() == ERROR)
    {
      res.response = getSubState();
    }

    ROS_INFO("[%s] Service reply with success: %s\n", getLimb().c_str(),
              res.success?"true":"false");
    return true;
}

bool BaxterArmCtrl::cancelCb(std_srvs::Trigger::Request  &req,
                             std_srvs::Trigger::Response &res)
{
  setState(KILLED);
  res.success = true;
  return true;
}

/** ACTION DB MAINTENANCE **/

bool BaxterArmCtrl::insertAction(const std::string &a,
                                 BaxterArmCtrl::f_action f,
                                 const std::string &target)
{
    if (a == LIST_ACTIONS)
    {
        ROS_ERROR("[%s][action_db] Attempted to insert protected action: %s",
                  getLimb().c_str(), a.c_str());
        return false;
    }

    if (isActionInDB(a)) // The action is in the db
    {
        ROS_WARN("[%s][action_db] Overwriting existing action with key %s",
                 getLimb().c_str(), a.c_str());
    }

    s_action row;
    row.call = f;
    row.target = target;

    action_db.insert( std::make_pair( a, row ));
    return true;
}

bool BaxterArmCtrl::removeAction(const std::string &a)
{
    if (isActionInDB(a)) // The action is in the db
    {
        action_db.erase(a);
        return true;
    }

    return false;
}

bool BaxterArmCtrl::callAction(const std::string &a)
{
    if (isActionInDB(a)) // The action is in the db
    {
        f_action act = action_db[a].call;
        return (this->*act)();
    }
    else
    {
      setSubState(ACT_RESP::_ACT_NOT_IN_DB);
      ROS_ERROR("[%s] Action %s is not in the database!",
		getLimb().c_str(), a.c_str());
    }

    return false;
}

bool BaxterArmCtrl::isActionInDB(const std::string &a)
{
    if (action_db.find(a) != action_db.end()) return true;
    return false;
}

void BaxterArmCtrl::printActionDB()
{
    ROS_INFO("[%s] Available actions in the database : %s",
              getLimb().c_str(), actionDBToString().c_str());
}

string BaxterArmCtrl::actionDBToString()
{
    string res = "";
    map<string, s_action>::iterator it;

    for ( it = action_db.begin(); it != action_db.end(); ++it )
    {
        res = res + it->first + ", ";
    }
    res = res.substr(0, res.size()-2); // Remove the last ", "
    return res;
}

void BaxterArmCtrl::setHomeConf(double s0, double s1, double e0, double e1,
				double w0, double w1, double w2)
{
    home_conf << s0, s1, e0, e1, w0, w1, w2;
    return;
}

/** CONTROL HELPER FUNCTIONS **/

bool BaxterArmCtrl::moveArm(string dir, double dist, string mode,
                            bool disable_coll_av)
{
    Point p_s = getPos();
    Point p_c;
    Point p_f;
    Point diff = Point();

    Quaternion o_f = getOri();


    if      (dir == "backward") diff.x -= 1;
    else if (dir == "forward")  diff.x += 1;
    else if (dir == "right")    diff.y -= 1;
    else if (dir == "left")     diff.y += 1;
    else if (dir == "down")     diff.z -= 1;
    else if (dir == "up")       diff.z += 1;
    else                         return false;

    p_f = p_s + diff * dist;

    ros::Time t_start = ros::Time::now();

    bool finish = false;

    ros::Rate r(100);
    while(RobotInterface::ok() && !isPositionReached(p_f, mode) &&
          not isClosing())
    {
        if (disable_coll_av)    suppressCollisionAv();

        double t_elap = (ros::Time::now() - t_start).toSec();
        p_c = p_s;

        if (!finish)
        {
   	    p_c = p_c + diff * getArmSpeed() * t_elap;
            if (dot(p_c - p_f, diff) >= 0) finish = true;
        }
        else
        {
            p_c = p_f;
        }

        if (!goToPoseNoCheck(p_c, o_f)) return false;

        r.sleep();
    }

    return true;
}

bool BaxterArmCtrl::goToPose(double px, double py, double pz,
                       double ox, double oy, double oz, double ow,
                       std::string mode, bool disable_coll_av)
{
    bool res = RobotInterface::goToPose(px, py, pz,
                                        ox, oy, oz, ow, mode, disable_coll_av);

    if (res == false && getSubState() != ACT_RESP::_ACT_KILLED)
    {
      setSubState(ACT_RESP::_INV_KIN_FAILED);
    }

    return res;
}

bool BaxterArmCtrl::releaseAtPose(double px, double py, double pz,
                                  double ox, double oy, double oz, double ow,
                                  string mode)
{
  ros::Time t_start = ros::Time::now();
  double z_start       =       getPos().z;
  int cnt_ik_fail      =                0;

  ros::Rate r(100);
  while(RobotInterface::ok())
  {
    double t_elap = (ros::Time::now() - t_start).toSec();

    // Move to release point bit by bit, make sure z does not exceed
    double x = px;
    double y = py;
    double z = (z < pz) ? pz : z_start - getArmSpeed() * t_elap;

    if (goToPoseNoCheck(x,y,z,VERTICAL_ORI))
    {
      cnt_ik_fail = 0;

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

bool BaxterArmCtrl::hoverAboveTable(double height, string mode,
                                    bool disable_coll_av)
{
    Point p = getPos();
    return goToPose(p.x, p.y, height, VERTICAL_ORI, mode, disable_coll_av);
}

bool BaxterArmCtrl::homePoseStrict(bool disable_coll_av)
{
    ROS_INFO("[%s] Going to home position strict..", getLimb().c_str());

    ros::Rate r(100);
    while(RobotInterface::ok() && !isConfigurationReached(home_conf) &&
          not isClosing())
    {
        if (disable_coll_av)    suppressCollisionAv();

        goToJointConfNoCheck(home_conf);

        r.sleep();
    }

    return true;
}

void BaxterArmCtrl::recoverFromError()
{
    Gripper::open();
    goHome();
}

/** ACTIONS **/

bool BaxterArmCtrl::notImplemented()
{
    ROS_ERROR("[%s] Action not implemented!", getLimb().c_str());
    return false;
}

bool BaxterArmCtrl::reachObject()
{
    ROS_ERROR("[%s] Reaching not implemented!", getLimb().c_str());
    return false;
}

bool BaxterArmCtrl::goHome()
{
  if (!homePoseStrict())
    return false;
  if (!goToPose(home_loc.x, home_loc.y, home_loc.z, VERTICAL_ORI))
    return false;
  return true;
}

bool BaxterArmCtrl::moveToLocation()
{
  Point tgt = tgt_location;
  return goToPose(tgt.x, tgt.y, tgt.z, VERTICAL_ORI);
}

bool BaxterArmCtrl::findObject()
{
  // Lookup object again to make sure position is up to date
  LookupObject srv;
  srv.request.id = getTargetObject().id;
  if (!lookup_obj_client.call(srv)) {
    ROS_ERROR("[%s] Failed to call service lookup_object!", getLimb().c_str());
    return false;
  }
  if (!srv.response.success) {
    setSubState(ACT_RESP::_OBJ_NOT_TRACKED);
    ROS_ERROR("[%s] Object %d not tracked!\n",
              getLimb().c_str(), srv.request.id);
    return false;
  }
  Point p = srv.response.object.position;
  // Hover above new location
  if (!goToPose(p.x, p.y, Z_FIND, VERTICAL_ORI)) return false;
  return true;
}

bool BaxterArmCtrl::offerObject()
{
  // Lookup object again to make sure position is up to date
  LookupObject srv;
  srv.request.id = getTargetObject().id;
  if (!lookup_obj_client.call(srv)) {
    ROS_ERROR("[%s] Failed to call service lookup_object!", getLimb().c_str());
    return false;
  }
  if (!srv.response.success) {
    setSubState(ACT_RESP::_OBJ_NOT_TRACKED);    
    ROS_ERROR("[%s] Object %d not tracked!\n",
              getLimb().c_str(), srv.request.id);
    return false;
  }
  Point p = srv.response.object.position;
  // Hover above last-remembered location and offer obj
  if (!goToPose(p.x, p.y, Z_FIND, VERTICAL_ORI)) {
    ROS_ERROR("[%s] Failed to go to object location!\n", getLimb().c_str());
    return false;
  }
  // Slowly rotate arm to face human
  // TODO: Fix this to work with Eigen::VectorXd
  // sensor_msgs::JointState j = getJointStates();
  // j.position[5] = -0.5;
  // ros::Rate r(100);
  // while(RobotInterface::ok() && !isConfigurationReached(j.position))
  // {
  //   goToJointConfNoCheck(j.position);
  //   r.sleep();
  // }
  // Sleep and wait for user input
  ros::Duration(3).sleep();
  return true;
}

bool BaxterArmCtrl::pickObject()
{
  // Check if object is currently held
  if (Gripper::is_gripping()) {
    setSubState(ACT_RESP::_OBJ_HELD);
    return false;
  }
  if (!findObject())               return false;
  if (!reachObject()) {
    goHome();
    setSubState(ACT_RESP::_OBJ_NOT_REACHED);
    return false;
  }
  if (!Gripper::close()) {
    setSubState(ACT_RESP::_OBJ_NOT_REACHED);
    return false;
  }
  setSubState(ACT_REQ::_ACTION_PICKUP);
  // Move up from current position to Z_LOW
  geometry_msgs::Point p = getPos();
  if (!goToPose(p.x, p.y, Z_LOW, VERTICAL_ORI)) return false;
  return true;
}

bool BaxterArmCtrl::putObject()
{
  // Check if object is currently held
  if (!Gripper::is_gripping()) {
    setSubState(ACT_RESP::_OBJ_NOT_HELD);
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

bool BaxterArmCtrl::replaceObject()
{
  // Check if object is currently held
  if (!Gripper::is_gripping()) {
    setSubState(ACT_RESP::_OBJ_NOT_HELD);
    return false;
  }
  // Move to location of last picked object and release
  if (!goToPose(last_pick_loc.x, last_pick_loc.y,
                Z_RELEASE, VERTICAL_ORI)) return false;
  ros::Duration(1).sleep();
  Gripper::open();

  return true;
}

bool BaxterArmCtrl::waitForFeedback()
{
  printf("[%s] Waiting for feedback...\n", getLimb().c_str());
  setState(WORKING);
  ros::Duration(3).sleep();
  return true;
}

/** MISCELLANEOUS **/

bool BaxterArmCtrl::setState(int _state)
{
  ROS_DEBUG("[%s] Setting state to %i", getLimb().c_str(), _state);
    
  if (_state == DONE)
    {
      setSubState(getAction());
    }
  else if (_state == ERROR && getSubState() == "")
    {
      setSubState(ACT_RESP::_ACT_FAILED);
    }
  else if (_state == KILLED)
    {
      setSubState(ACT_RESP::_ACT_KILLED);
    }

  return RobotInterface::setState(_state);
}

void BaxterArmCtrl::setSubState(const string& _sub_state)
{
    sub_state =  _sub_state;
}

bool BaxterArmCtrl::setAction(const string& _action)
{
    setPrevAction(getAction());
    action = _action;
    publishState();

    return true;
}

bool BaxterArmCtrl::setPrevAction(const string& _prev_action)
{
    prev_action = _prev_action;

    return true;
}

bool BaxterArmCtrl::setArmSpeed(double _arm_speed)
{
  arm_speed = _arm_speed;

  return true;
}

bool BaxterArmCtrl::publishState()
{
    human_robot_collaboration_msgs::ArmState msg;

    msg.state  = string(getState());
    msg.action = getAction();
    msg.object = to_string(getTargetObject().id);

    state_pub.publish(msg);

    return true;
}

BaxterArmCtrl::~BaxterArmCtrl()
{
    setIsClosing(true);
    if (arm_thread.joinable()) { arm_thread.join(); }
}
