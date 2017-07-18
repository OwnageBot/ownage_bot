#include "baxter_arm_ctrl.h"

using namespace std;
using namespace geometry_msgs;
using namespace baxter_core_msgs;
using namespace ownage_bot;

BaxterArmCtrl::BaxterArmCtrl(string _name, string _limb, bool _use_robot, bool _use_forces, bool _use_trac_ik, bool _use_cart_ctrl) :
                 RobotInterface(_name,_limb, _use_robot, THREAD_FREQ, _use_forces, _use_trac_ik, _use_cart_ctrl),
                 Gripper(_limb, _use_robot), sub_state(""), action(""), prev_action(""), sel_object_id(-1)
{
    std::string other_limb = getLimb() == "right" ? "left" : "right";

    std::string topic = "/"+getName()+"/service_"+_limb;
    service = nh.advertiseService(topic, &BaxterArmCtrl::serviceCb, this);
    ROS_INFO("[%s] Created service server with name  : %s", getLimb().c_str(), topic.c_str());

    insertAction(ACTION_HOME, &BaxterArmCtrl::goHome, "none");
    insertAction(ACTION_RELEASE, &BaxterArmCtrl::openImpl, "none");
}

bool BaxterArmCtrl::startThread()
{
    // This allows to call multiple threads within the same thread object.
    // As written in http://en.cppreference.com/w/cpp/thread/thread/joinable :
    //      A thread that has finished executing code, but has not yet been joined is still considered
    //      an active thread of execution and is therefore joinable.
    // So, we need to join the thread in order to spun out a new one anyways.
    if (arm_thread.joinable())    { arm_thread.join(); };

    arm_thread = std::thread(&BaxterArmCtrl::InternalThreadEntry, this);
    return arm_thread.joinable();
}

void BaxterArmCtrl::InternalThreadEntry()
{
    nh.param<bool>("internal_recovery",  internal_recovery, true);
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
    else if (a == ACTION_HOME || a == ACTION_RELEASE)
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
        ROS_ERROR("[%s] Invalid Action %s in state %i", getLimb().c_str(), a.c_str(), s);
    }

    if (int(getState())==WORKING)
    {
        setState(ERROR);
    }

    if (int(getState())==ERROR)
    {
        ROS_ERROR("[%s] Action %s not successful! State %s %s", getLimb().c_str(), a.c_str(),
                                          string(getState()).c_str(), getSubState().c_str());
    }

    return;
}

bool BaxterArmCtrl::serviceCb(CallAction::Request  &req, CallAction::Response &res)
{
    // Let's read the requested action and object to act upon
    setSubState("");
  
    string action = req.action;

    ROS_INFO("[%s] Service request received. Action: %s", getLimb().c_str(), action.c_str());

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
        ROS_ERROR("[%s] Action %s is not in the database!", getLimb().c_str(), action.c_str());
        res.success = false;
        res.response = ACT_NOT_IN_DB;
        return true;
    }
  
    setAction(action);
    string target = action_db[action].target;

    if (target == "object")
    {
        setTargetObject(req.object);
    }
    else if (target == "location")
    {
        setTargetLocation(req.location)
    }

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
            res.response = ACT_FAILED;
            break;
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

bool BaxterArmCtrl::notImplemented()
{
    ROS_ERROR("[%s] Action not implemented!", getLimb().c_str());
    return false;
}

bool BaxterArmCtrl::insertAction(const std::string &name, BaxterArmCtrl::f_action f,
                                 const std::string &target)
{
    if (a == LIST_ACTIONS)
    {
        ROS_ERROR("[%s][action_db] Attempted to insert protected action key: %s",
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
        setSubState(ACT_NOT_IN_DB);
        ROS_ERROR("[%s] Action %s is not in the database!", getLimb().c_str(), a.c_str());
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

bool BaxterArmCtrl::moveArm(string dir, double dist, string mode, bool disable_coll_av)
{
    Point p_s = getPos();
    Point p_c = getPos();
    Point p_f = getPos();

    Quaternion o_f = getOri();

    if      (dir == "backward") p_f.x -= dist;
    else if (dir == "forward")  p_f.x += dist;
    else if (dir == "right")    p_f.y -= dist;
    else if (dir == "left")     p_f.y += dist;
    else if (dir == "down")     p_f.z -= dist;
    else if (dir == "up")       p_f.z += dist;
    else                         return false;

    ros::Time t_start = ros::Time::now();

    bool finish = false;

    ros::Rate r(100);
    while(RobotInterface::ok() && !isPositionReached(p_f, mode) && not isClosing())
    {
        if (disable_coll_av)    suppressCollisionAv();

        double t_elap = (ros::Time::now() - t_start).toSec();
        p_c = p_s;

        if (!finish)
        {
            if (dir == "backward" || dir == "forward")
            {
                int sgn = dir=="backward"?-1:+1;
                p_c.x = p_c.x + sgn * ARM_SPEED * t_elap;

                if (dir == "backward")
                {
                    if (p_c.x < p_f.x) finish = true;
                }
                else if (dir == "forward")
                {
                    if (p_c.x > p_f.x) finish = true;
                }
            }
            if (dir == "right" || dir == "left")
            {
                int sgn = dir=="right"?-1:+1;
                p_c.y = p_c.y + sgn * ARM_SPEED * t_elap;

                if (dir == "right")
                {
                    if (p_c.y < p_f.y) finish = true;
                }
                else if (dir == "left")
                {
                    if (p_c.y > p_f.y) finish = true;
                }
            }
            if (dir == "down" || dir == "up")
            {
                int sgn = dir=="down"?-1:+1;
                p_c.z = p_c.z + sgn * ARM_SPEED * t_elap;

                if (dir == "down")
                {
                    if (p_c.z < p_f.z) finish = true;
                }
                else if (dir == "up")
                {
                    if (p_c.z > p_f.z) finish = true;
                }
            }
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

    if (res == false && getSubState() != ACT_FAILED)
    {
        setSubState(INV_KIN_FAILED);
    }

    return res;
}

bool BaxterArmCtrl::hoverAboveTable(double height, string mode, bool disable_coll_av)
{
    if (getLimb() == "right")
    {
        return goToPose(HOME_POS_R, height, VERTICAL_ORI_R,
                                    mode, disable_coll_av);
    }
    else if (getLimb() == "left")
    {
        return goToPose(HOME_POS_L, height, VERTICAL_ORI_L,
                                    mode, disable_coll_av);
    }
    else return false;
}

bool BaxterArmCtrl::homePoseStrict(bool disable_coll_av)
{
    ROS_INFO("[%s] Going to home position strict..", getLimb().c_str());

    ros::Rate r(100);
    while(RobotInterface::ok() && !isConfigurationReached(home_conf) && not isClosing())
    {
        if (disable_coll_av)    suppressCollisionAv();

        goToJointConfNoCheck(home_conf);

        r.sleep();
    }

    return true;
}

void BaxterArmCtrl::setHomeConf(double s0, double s1, double e0, double e1,
                                     double w0, double w1, double w2)
{
    home_conf.clear();
    home_conf.push_back(s0);
    home_conf.push_back(s1);
    home_conf.push_back(e0);
    home_conf.push_back(e1);
    home_conf.push_back(w0);
    home_conf.push_back(w1);
    home_conf.push_back(w2);

    return;
}

bool BaxterArmCtrl::goHome()
{
    return homePoseStrict();
}

void BaxterArmCtrl::recoverFromError()
{
    Gripper::open();
    goHome();
}

bool BaxterArmCtrl::setState(int _state)
{
    ROS_DEBUG("[%s] Setting state to %i", getLimb().c_str(), _state);

    if (_state == KILLED && getState() != WORKING)
    {
        ROS_WARN_THROTTLE(2, "[%s] Attempted to kill a non-working controller", getLimb().c_str());
        return false;
    }

    if      (_state == DONE)
    {
        setSubState(getAction());
    }
    else if (_state == ERROR && getSubState() == "")
    {
        setSubState(ACT_FAILED);
    }

    return RobotInterface::setState(_state);
}

void BaxterArmCtrl::setSubState(const string& _sub_state)
{
    // ROS_DEBUG("[%s] Setting sub state to: %s", getLimb().c_str(), _sub_state.c_str());
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

bool BaxterArmCtrl::publishState()
{
    human_robot_collaboration_msgs::ArmState msg;

    msg.state  = string(getState());
    msg.action = getAction();
    msg.object = getObjectNameFromDB(getObjectID());

    state_pub.publish(msg);

    return true;
}

BaxterArmCtrl::~BaxterArmCtrl()
{
    setIsClosing(true);
    if (arm_thread.joinable()) { arm_thread.join(); }
}
