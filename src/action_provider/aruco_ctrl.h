#ifndef __ARUCO_CTRL_H__
#define __ARUCO_CTRL_H__

/**
 * Specializes BaxterArmCtrl to reach for objects using ARuco tag data.
 * Inherits from ARucoClient to access ARuco tag data.
 */

#include <string>
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/Trigger.h>
#include "robot_perception/aruco_client.h"
#include "baxter_arm_ctrl.h"

// Offset between gripper and IR sensor so that picking up works
#define IR_OFFSET (0.015)
// Force threshold for releasing object
#define REACH_THRESHOLD (-15)

class ARucoCtrl : public BaxterArmCtrl, public ARucoClient
{
private:
    // Subscriber to the ARuco detector,
    ros::Subscriber aruco_sub;

protected:

    /**
     * Reach for target object. Specialized from BaxterArmCtrl so as to locate
     * object via its ARuco tag.
     * @return true/false if success/failure
     */
    bool reachObject();

    /**
     * Sets default joint-level configuration for the home position
     */
    void setHomeConfiguration();

   /**
     * Sets the corner positions for the workspace
     */
    void setWorkspaceConfiguration();

public:

    /**
     * Constructor
     */
    ARucoCtrl(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~ARucoCtrl();
};

#endif
