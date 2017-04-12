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
#include "ownage_bot/LocateObject.h"
#include "ownage_bot/RichObject.h"

#define ACTION_FIND "find"
#define ACTION_SCAN "scan"
#define ACTION_PUT "put"

#define ACT_CANCELLED "Action cancelled by user"

#define RELEASE_HEIGHT (0.1)

class ObjectPicker : public ArmCtrl, public ARucoClient
{
private:
    double elap_time;

    // Arm pose configurations for corners of physical workspace
    std::vector< std::vector<double> > workspace_conf;
    
    // Subscriber to the ARuco detector,
    ros::Subscriber _aruco_sub;

    // Subscriber to the new object topic,
    ros::Subscriber _new_obj_sub;

    // Client for LocateObject service
    ros::ServiceClient _loc_obj_client;

    /**
     * Picks up object with ARuco tag, where the id is first set
     * by calling setObjectID
     * @return true/false if success/failure
     */
    bool pickARTag();

    /**
     * Finds the object with id specified by setObjectID by checking
     * the ObjectTracker node for its location, then moving the arm
     * such that the object is within camera view
     * @return true/false if success/failure
     */
    bool findObject();

    /**
     * Moves to home pose to reset inverse kinematics, calls pickARTag
     * then moves arm up slightly while holding the picked object
     * @return true/false if success/failure
     */
    bool pickObject();

    /**
     * Moves arm down close to the table, then releases object in place
     * @return true/false if success/failure
     */
    bool putObject();
    
    /**
     * Moves arm around the workspace so that the camera can gather data
     * @return true/false if success/failure
     */
    bool scanWorkspace();

    /**
     * Recovers from errors during execution. Releases object then moves
     * back to the home position
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

    /**
     * Callback function for service. Two main changes from ArmCtrl:
     * - Only requires objects to be specified for the 'get' action
     * - Sends ACT_CANCELLED instead if the action is stopped by the human
     */
    bool serviceCb(baxter_collaboration_msgs::DoAction::Request  &req,
                   baxter_collaboration_msgs::DoAction::Response &res);

    /**
     * Sets current object ID to be picked up and manipulated
     */
    void setObjectID(int _obj);
};

#endif
