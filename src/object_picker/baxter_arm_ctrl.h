/**
 * Copyright (C) 2017 Social Robotics Lab, Yale University
 * Modified by: Tan Zong Xuan
 * Original author: Alessandro Roncone (alessandro.roncone@yale.edu)
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2.1 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

#ifndef __BAXTER_ARM_CTRL_H__
#define __BAXTER_ARM_CTRL_H__

#include <map>
#include <thread>
#include <mutex>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>

#include "robot_interface/robot_interface.h"
#include "robot_interface/gripper.h"

#include "ownage_bot/ObjectMsg.h"
#include "ownage_bot/CallAction.h"

class BaxterArmCtrl : public RobotInterface, public Gripper
{
private:
    // Substate of the controller (useful to keep track of
    // long actions that need multiple internal states)
    std::string sub_state;

    // High level action the controller is engaged in
    std::string action;
    // Previous high level action (for complex actions)
    std::string prev_action;

    // Target object for action
    ownage_bot::ObjectMsg tgt_object;
    // Target location for action
    geometry_msg::Point tgt_location;

    // Flag to know if the robot will try to recover from an error
    // or will wait the external planner to take care of that
    bool internal_recovery;

    // Service to request actions to
    ros::ServiceServer  service;

    // Home configuration. Setting it in any of the children
    // of this class is mandatory (through the virtual method
    // called setHomeConfiguration() )
    std::vector<double> home_conf;

    // Internal thread functionality
    std::thread arm_thread;

    /**
     * Provides basic functionalities for the object, such as a goHome and open.
     * For deeper, class-specific specialization, please modify doAction() instead.
     */
    void InternalThreadEntry();

    /**
     * Wrapper for Gripper:open() so that it can fit the action_db specifications
     * in terms of function signature.
     *
     * @return true/false if success/failure
     */
    bool openImpl() { return open(); }

protected:

    /**
     * Pointer to the action prototype function, which does not take any
     * input argument and returns true/false if success/failure
     */
    typedef bool(ArmCtrl::*f_action)();

    /**
     * Structure that stores action prototype and other metadata
     */
    struct s_action {
        // Action protype function
        f_action call;
        // Target type, "none", "object" or "location"
        std::string target;
    };

    /**
     * Action database, which pairs a string key, corresponding to the action name,
     * with its relative action, which is an f_action.
     *
     * Please be aware that, by default, if the user calls an action with the wrong
     * key or an action that is not available, the code will segfault. By C++
     * standard: operator[] returns (*((insert(make_pair(x, T()))).first)).second
     * Which means that if we are having a map of pointers to functions, a wrong key
     * will segfault the software. A layer of protection has been put in place to
     * avoid accessing a non-existing key (so this does not happen any more, but it
     * is still worth knowing).
     */
    std::map <std::string, s_action> action_db;

    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    virtual void recoverFromError();

    /**
     * Hovers above table at a specific x-y position.
     * @param  height the z-axis value of the end-effector position
     * @return        true/false if success/failure
     */
    bool hoverAboveTable(double height, std::string mode="loose",
                         bool disable_coll_av = false);

    /**
     * Home position with a specific joint configuration. This has
     * been introduced in order to force the arms to go to the home configuration
     * in always the same exact way, in order to clean the seed configuration in
     * case of subsequent inverse kinematics requests.
     *
     * @param  disable_coll_av if to disable the collision avoidance while
     *                         performing the action or not
     * @return                 true/false if success/failure
     */
    bool homePoseStrict(bool disable_coll_av = false);

    /**
     * Sets the joint-level configuration for the home position
     *
     * @param s0 First  shoulder joint
     * @param s1 Second shoulder joint
     * @param e0 First  elbow    joint
     * @param e1 Second elbow    joint
     * @param w0 First  wrist    joint
     * @param w1 Second wrist    joint
     * @param w2 Third  wrist    joint
     */
    void setHomeConf(double s0, double s1, double e0, double e1,
                                double w0, double w1, double w2);

    /**
     * Sets the joint-level configuration for the home position
     */
    virtual void setHomeConfiguration() { return; };

    /**
     * Goes to the home position, and "releases" the gripper
     *
     * @return        true/false if success/failure
     */
    bool goHome();

    /**
     * Moves arm in a direction requested by the user, relative to the current
     * end-effector position
     *
     * @param dir  the direction of motion (left right up down forward backward)
     * @param dist the distance from the end-effector starting point
     *
     * @return true/false if success/failure
     */
    bool moveArm(std::string dir, double dist, std::string mode = "loose",
                 bool disable_coll_av = false);

    /*
     * Moves arm to the requested pose , and checks if the pose has been achieved.
     * Specializes the RobotInterface::gotoPose method by setting the sub_state to
     * INV_KIN_FAILED if the method returns false.
     *
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @param  mode (either loose or strict, it checks for the final desired position)
     * @return true/false if success/failure
     */
    bool goToPose(double px, double py, double pz,
                  double ox, double oy, double oz, double ow,
                  std::string mode="loose", bool disable_coll_av = false);

    /**
     * Placeholder for an action that has not been implemented (yet)
     *
     * @return false always
     */
    bool notImplemented();

    /**
     * Adds an action to the action database
     *
     * @param   name the action to be inserted
     * @param   f a pointer to the action, in the form bool action()
     * @param   target "none", "object" or "location", the target type of the action
     * @return    true/false if the insertion was successful or not
     */
    bool insertAction(const std::string &name, BaxterArmCtrl::f_action f,
                      const std::string &target);

    /**
     * Removes an action from the database. If the action is not in the
     * database, the return value will be false.
     *
     * @param   a the action to be removed
     * @return    true/false if the removal was successful or not
     */
    bool removeAction(const std::string &a);

    /**
     * Calls an action from the action database
     *
     * @param    a the action to take
     * @return     true/false if the action called was successful or failed
     */
    bool callAction(const std::string &a);

    /**
     * Checks if an action is available in the database
     * @param             a the action to check for
     * @return   true/false if the action is available in the database
     */
    bool isActionInDB(const std::string &a);

    /**
     * Prints the action database to screen.
     */
    void printActionDB();

    /**
     * Converts the action database to a string.
     * @return the list of allowed actions, separated by a comma.
     */
    std::string actionDBToString();

    /**
     * Publishes the high-level state of the controller (to be shown in the baxter display)
     */
    bool publishState();

    /**
     * Sets the previous action to (usually) the last action that has been requested.
     *
     * @param _prev_action the last action that has been requested
     * @return             true/false if success/failure
     */
    bool setPrevAction(const std::string& _prev_action);

    /**
     * Sets the sub state to a new sub state
     *
     * @param _sub_state the new sub state
     */
    virtual void setSubState(const std::string& _sub_state);

public:
    /**
     * Constructor
     */
    BaxterArmCtrl(std::string        _name, std::string           _limb,
                  bool   _use_robot = true, bool    _use_forces =  true,
                  bool _use_trac_ik = true, bool _use_cart_ctrl = false);

    /*
     * Destructor
     */
    virtual ~BaxterArmCtrl();

    /**
     * Starts thread that executes the control server.
     */
    bool startThread();

    /**
     * Callback for the service that requests actions
     * @param  req the action request
     * @param  res the action response (res.success either true or false)
     * @return     true always :)
     */
    bool serviceCb(ownage_bot::CallAction::Request  &req,
                   ownage_bot::CallAction::Response &res);

    /**
     * Callback function for cancel service
     */
    bool cancelCb(std_srvs::Trigger::Request  &req,
                  std_srvs::Trigger::Response &res);

    /* Self-explaining "setters" */
    void setTargetObject(ownage_bot::ObjectMsg& _obj) { tgt_object =  _obj; };
    void setTargetLocation(geometry_msgs::Point& _p) { tgt_location =  _p; };

    /**
     * Sets the action
     *
     * @param _action the new action
     * @return        true/false if success/failure
     */
    bool setAction(const std::string& _action);

    /*
     * Sets the internal state.
     *
     * @return true/false if success/failure
     */
    bool setState(int _state);

    /* Self-explaining "getters" */
    std::string       getSubState() { return         sub_state; };
    std::string         getAction() { return            action; };
    std::string     getPrevAction() { return       prev_action; };
    
    ownage_bot::ObjectMsg getTargetObject() { return tgt_object; };
    geometry_msgs::Point getTargetLocation() { return tgt_location; };
    
    bool      getInternalRecovery() { return internal_recovery; };
};

#endif
