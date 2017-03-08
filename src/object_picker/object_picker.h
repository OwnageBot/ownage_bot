#ifndef __OBJECT_PICKER_H__
#define __OBJECT_PICKER_H__

/** 
 * Modified from CubePicker.h in baxter_collaboration 
 * Uses Baxter's left arm to pick up and put down objects via suction. 
 * Inherits from ArmCtrl and ARucoClient, so look there for more method declarations.
 */

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <baxter_collaboration_lib/robot_interface/arm_ctrl.h>
#include <baxter_collaboration_lib/robot_perception/aruco_client.h>

class ObjectPicker : public ArmCtrl, public ARucoClient
{
private:
    double elap_time;

    // Subscriber to the ARuco detector,
    ros::Subscriber _aruco_sub;

    /**
     * [moveObjectTowardHuman description]
     * @return true/false if success/failure
     */
    bool moveObjectTowardHuman();

    /**
     * [pickARTag description]
     * @return true/false if success/failure
     */
    bool pickARTag();

    /**
     * [pickObject description]
     * @return true/false if success/failure
     */
    bool pickObject();

    /**
     * [passObject description]
     * @return true/false if success/failure
     */
    bool passObject();

    /**
     * [pickObject description]
     * @return true/false if success/failure
     */
    bool pickPassObject();

    /**
     * [recoverPickPass description]
     * @return true/false if success/failure
     */
    bool recoverPickPass();

    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    void recoverFromError();

    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

protected:

    /**
     * Adds new objects to the object database when notified by the object tracker node
     */
    void newObjectCallback(const std_msgs::UInt32 msg);

public:
    /**
     * Constructor
     */
    ObjectPicker(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~ObjectPicker();

    void setObjectID(int _obj);
};

#endif
