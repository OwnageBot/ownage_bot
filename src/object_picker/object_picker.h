#ifndef __OBJECT_PICKER_H__
#define __OBJECT_PICKER_H__

/**
 * Modified from CubePicker.h in baxter_collaboration
 * Uses Baxter's left arm to pick up and put down objects via suction.
 * Inherits from ArmCtrl and ARucoClient, so look there for more method declarations.
 */

#include <string>
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include "robot_interface/arm_ctrl.h"
#include "robot_perception/aruco_client.h"

#define ACTION_SCAN "scan"
#define ACTION_PUT "put"

#define RELEASE_HEIGHT (0.1)

class ObjectPicker : public ArmCtrl, public ARucoClient
{
private:
    double elap_time;

    // Arm pose configurations for corners of physical workspace
    std::vector< std::vector<double> > workspace_conf;
    
    // Subscriber to the ARuco detector,
    ros::Subscriber _aruco_sub;

    // Subscriber to the object tracker,
    ros::Subscriber _new_obj_sub;

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
     * [putObject description]
     * @return true/false if success/failure
     */
    bool putObject();
    
    /**
     * [scanWorkspace description]
     * @return true/false if success/failure
     */
    bool scanWorkspace();

    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    void recoverFromError();

    /**
     * Sets default joint-level configuration for the home position
     */
    void setHomeConfiguration();

   /**
     * Sets default joint-level configuration for the workspace corners
     */
    void setWorkspaceConfiguration();

    
protected:
    
    /**
     * Adds new objects to the object database when notified by the object
     * tracker node.
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
