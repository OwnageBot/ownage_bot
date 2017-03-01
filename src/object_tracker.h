#ifndef __OBJECT_TRACKER_H__
#define __OBJECT_TRACKER_H__

class RichObject
{
public:
    // ID of the object's marker
    int id;
    
    // Object position and orientation (with respect to camera)
    geometry_msgs::Point        local_pos;
    geometry_msgs::Quaternion   local_ori;
    
    // Color
    int[3] HSV;
    
    // Last contact time (ms)
    int last_contact_time;
    
    // Forbiddenness
    int forbiddenness;   
};

class ObjectTracker : public ARucoClient
{
protected:
    /**
     * Callback function for the ARuco topic
     * @param msg the topic message
     */
    void ARucoCb(const aruco_msgs::MarkerArray& msg);

    std::vector<RichObject> object_db;

public:
    /**
     * Constructor
     */
    ObjectTracker(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~ObjectTracker();

};

#endif
