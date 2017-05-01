# OwnageBot
A robot that learns the rules of ownership based on interaction with objects and agents in its environment.

README adapted from the [Baxter Collaboration repository](https://github.com/ScazLab/baxter_collaboration).

## Prerequisites

* `aruco_ros`: for recognition and tracking of QR codes
* `baxter_collaboration_lib`: for high-level arm control

## Compilation

We follow [`baxter_collaboration`](https://github.com/ScazLab/baxter_collaboration) in using `catkin_tools`. To compile:

1. Make sure you're on the correct branch/version of both `baxter_collaboration_lib` and `aruco_ros`
2. Compile `baxter_collaboration_lib` and `aruco_ros` if necessary
3. Compile ownage_bot: `catkin build ownage_bot`

## Execution

### Initial steps (mainly for Scazlab researchers)

0. Turn on the robot. Wait for the robot to finish its start-up phase.
1. Be sure that the system you're running the code has access to the Baxter robot. This is usually done by running the `baxter.sh` script that should be provided in your Baxter installation.
2. Untuck the robot. **@ScazLab students** → we have an alias for this, so you just have to type `untuck`.

### Automated collection

Intended mode of behavior, where Baxter repeatedly tries to pick up all objects in its workspace and bring them within its designated home area. Negative feedback can be given after it picks up the object, after which it will ask each detected agent in the workspace whether it owns the held object. If an agent claims the object, Baxter will put the object back in its original location, otherwise it will assume the negative feedback was mistaken and put the object in its home area.

1. Run `roslaunch ownage_bot.launch`
2. Watch as Baxter tries to collect all QR-tagged objects in the workspace.
3. Wait until it brings an object above its home area before giving feedback.
4. Apply negative feedback by pressing any cuff button (3 second window).
5. Wait for Baxter to offer object to each agent's avatar in turn.
6. Reject the offer by pressing any cuff button. Accept by simply waiting.
7. Repeat as necessary.

### Manual arm control

For testing and debugging the arm control services provided by [`object_picker`](https://github.com/OwnageBot/ownage_bot/tree/master/src/object_picker)

1. Run `roslaunch ownage_bot.launch is_manual:=true`
2. Request an action by calling `rosservice call /action_provider/service_left "{action: 'action_name', objects: [list of ids]}"`

#### List of supported actions (left arm only)

* `list_actions`: returns a list of supported actions
* `home`: moves the arm to a position above its home area
* `scan`: scans the workspace for object changes
* `find`: moves arm over the location of specified object (requires object ID)
* `get`: picks up an object with a vacuum gripper (requires object ID)
* `put`: puts down an object gently at current x-y location
* `release`: turns off vacuum gripper at current height and location
* `replace`: replaces object in last pick-up location
* `wait`: waits 3 seconds, can be interrupted by feedback from cuff button

### Simulated learning

For simulated training and testing of [`object_classifier`](https://github.com/OwnageBot/ownage_bot/tree/master/src/object_classifier)

1. Run `roslaunch ownage_bot.launch is_simulation:=true`
2. Wait for the results to be printed in the terminal

## Architecture

### Nodes

OwnageBot is comprised by five different modules, each of which runs as a ROS node to provide a certain functionality:

* [`object_tracker`](https://github.com/OwnageBot/ownage_bot/tree/master/src/object_tracker) - tracks all ARuco QR-tagged objects in the workspace, determining their absolute position (relative to Baxter's base frame), color, and proximity to avatars.
* [`object_picker`](https://github.com/OwnageBot/ownage_bot/tree/master/src/object_picker) - provides high-level arm control as ROS services by inheriting the ArmCtrl interface in [`baxter_collaboration_lib`](https://github.com/ScazLab/baxter_collaboration/tree/master/baxter_collaboration_lib).
* [`object_collector`](https://github.com/OwnageBot/ownage_bot/tree/master/src/object_collector) - contains the main autonomous loop, repeatedly calling services provided by `object_picker` in order to scan the workspace and collect any unowned objects (where ownership is determined by `object_classifier`).
* [`object_classifier`](https://github.com/OwnageBot/ownage_bot/tree/master/src/object_classifier) - receives labelled example data from `object_collector` whenever an object thought to be unowned is claimed by another agent, which is then used to train the classifier.
* [`object_tester`](https://github.com/OwnageBot/ownage_bot/tree/master/src/object_tester) - generates virtual objects, avatars and examples, in order to train and test the learning algorithm in `object_classifier`.

### Topics & Services

When using `ownage_bot.launch`, these topic and service names are contained with the namespace `/ownage_bot` unless otherwise specified.

* `new_object` (topic)
  * Type: `uint32` (unsigned int)
  * Publisher(s): `object_tracker`
  * Subscriber(s): `object_picker`
  * Whenever `object-tracker` sees an ID that it has never seen before, it publishes to this topic.
* `feedback` (topic)
  * Type: [`RichFeedback`](https://github.com/OwnageBot/ownage_bot/blob/master/msg/RichFeedback.msg)
  * Publisher(s): `object_collector`, `object_tester` (in simulation)
  * Subscriber(s): `object_classifier`
  * Whenever Baxter successfully claims an object for itself, or is stopped and told the owner of the object, `object_collector` sends feedback to `object_classifier` as training data, using this topic. The three pieces of information contained in the feedback data structure are the *time of interaction*, *owner label* (avatar id of owner, 0 if unowned), and the *object data* itself.
* `locate_object` (service)
  * Type: [`LocateObject`](https://github.com/OwnageBot/ownage_bot/blob/master/srv/LocateObject.srv)
  * Server(s): `object_tracker`
  * Client(s): `object_picker`
  * Used when `object_picker` is performing the `find` action to locate an object that is currently out of the camera view.
* `list_objects` (service)
  * Type: [`ListObjects`](https://github.com/OwnageBot/ownage_bot/blob/master/srv/ListObjects.srv)
  * Server(s): `object_tracker`, `object_tester` (in simulation)
  * Client(s): `object_classifier`
  * Returns, without input, the list of objects currently tracked by `object_tracker`, with their most recently known properties.
* `classify_objects` (service)
  * Type: [`ListObjects`](https://github.com/OwnageBot/ownage_bot/blob/master/srv/ListObjects.srv)
  * Server(s): `object_classifier`
  * Client(s): `object_collector`, `object_tester` (in simulation)
  * Returns, without input, a list of all currently tracked objects classified according to ownership. More precisely, each object is returned together with a list of possible owners and corresponding ownership probabilities.
* `/action_provider/service_left` (service, *not in `ownage_bot` namespace*)
  * Type: [`DoAction`](https://github.com/ScazLab/baxter_collaboration/blob/master/baxter_collaboration_msgs/srv/DoAction.srv)
  * Server(s): `object_picker`
  * Client(s): `object_collector`
  * Used to perform various high-level actions using Baxter's left arm. Takes the action name and a list of objects as input, returns whether the action was successful along with a response string (e.g. an error message). See above for a list of supported actions.
