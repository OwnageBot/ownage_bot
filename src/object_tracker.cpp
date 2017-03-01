#include "object_tracker.h"

using namespace std;
using namespace baxter_core_msgs;

ObjectTracker::ObjectTracker(std::string _name, std::string _limb, bool _no_robot) :
                            ARucoClient(_name, _limb), elap_time(0)
{
}
