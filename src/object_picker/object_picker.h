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

// The names of the actions provided
#define ACTION_SCAN "scan"
#define ACTION_FIND "find"
#define ACTION_PUT "put"
#define ACTION_OFFER "offer"
#define ACTION_REPLACE "replace"
#define ACTION_WAIT "wait"

// Additional error messages that the service can report
#define ACT_CANCELLED "Action cancelled by user"
#define OBJECT_HELD "An object is already being held"
#define NO_OBJECT_HELD "No object is currently being held"

// Height at which objects are released when put down
#define Z_RELEASE (-0.1)
#define Z_FIND (0.07)

// Offset between gripper and IR sensor so that picking up works
#define IR_OFFSET (0.015)

// Force threshold for releasing object
#define RELEASE_THRESHOLD (-15)

class ObjectPicker : public ArmCtrl, public ARucoClient
{
private:
    double elap_time;

    // True if object is currently held
    bool is_holding;

    // Location of last picked object
    geometry_msgs::Point _last_pick_loc;

    // Endpoint position for home location
    geometry_msgs::Point home_loc;

    // Endpoint positions for corners of physical workspace
    std::vector< std::vector<double> > workspace_conf;

    // Subscriber to the ARuco detector,
    ros::Subscriber _aruco_sub;

    // Subscriber to the new object topic,
    ros::Subscriber _new_obj_sub;

    // Client for LocateObject service
    ros::ServiceClient _loc_obj_client;

protected:

    /**
     * Adds new objects to the object database when notified by the object
     * tracker node.
     */
    void newObjectCallback(const std_msgs::UInt32 msg);

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
     * Nearly identical to findObject, except that baxter
     * Turns hand up so as to offer an avtar the object
     * its holding
     */
    bool offerObject();

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
     * Replace currently held object in location it was picked up from
     * @return true/false if success/failure
     */
    bool replaceObject();

    /**
     * Moves arm around the workspace so that the camera can gather data
     * @return true/false if success/failure
     */
    bool scanWorkspace();

    /**
     * Waits a set amount of time for negative feedback before returning
     * @return true/false if success/failure
     */
    bool waitForFeedback();

    /**
     * Goes to the home position
     *
     * @return        true/false if success/failure
     */
    bool goHome();

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

    /*
     * Releases currently held object at pose, or upon collision (high wrench)
     *
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @param  mode (either loose or strict, it checks for the final desired position)
     * @return true/false if success/failure
     */
    bool releaseAtPose(double px, double py, double pz,
                       double ox, double oy, double oz, double ow,
                       std::string mode="loose");

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
