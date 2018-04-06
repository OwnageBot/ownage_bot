# OwnageBot
A robot that learns the rules of ownership based on interaction with objects and agents in its environment.
## Prerequisites

### Python prerequisites
* `scikit-learn`: for ownership inference via logistic regression
* `numpy`: for inference related math operations

### Additional prerequisites for use on Baxter
* `aruco_ros`: for recognition and tracking of QR codes
* `cv_bridge`: for computer vision via OpenCV
* `human_robot_collaboration_lib`: for high-level arm control

## Compilation

We follow [`human_robot_collaboration`](https://github.com/ScazLab/human_robot_collaboration) in using `catkin_tools`.

### Full compile

1. Make sure you're on the correct branch/version of both `human_robot_collaboration_lib` and `aruco_ros`
2. Compile `human_robot_collaboration_lib` and `aruco_ros` if necessary
3. Compile `ownage_bot`: `catkin build ownage_bot`

### Simulation-only compile

If you have a ROS installation but don't have `human_robot_collaboration` or `aruco_ros`, you can still compile and run the learning algorithm in
simulated mode.

1. Set the `OWNAGE_BOT_SIMULATION` variable: `export OWNAGE_BOT_SIMULATION=1`
  * Switch back to full compile by calling `unset OWNAGE_BOT_SIMULATION`
2. Compile `ownage_bot`: `catkin build ownage_bot`
  * You may have to delete `build/CMakeCache.txt` in your Catkin workspace for changes in the environment variables to be noticed.

## Execution

### Initial steps (mainly for Scazlab researchers)

0. Turn on the robot. Wait for the robot to finish its start-up phase.
1. Be sure that the system you're running the code has access to the Baxter robot. This is usually done by running the `baxter.sh` script that should be provided in your Baxter installation.
2. Untuck the robot. **@ScazLab students** → we have an alias for this, so you just have to type `untuck`.

### Command prompt mode

Running `roslaunch ownage_bot.launch` brings up a command prompt for text-based user input. Input can consist of atomic actions, higher-level tasks, permission-based instruction (i.e. forbidding actions on specific objects), rule-based instruction (i.e. forbidding actions on based on object properties), listing and clearing various databases, etc. A detailed list of input commands is given below.

#### List of supported input commands



### Manual arm control

For testing and debugging the arm control services provided by [`object_picker`](https://github.com/OwnageBot/ownage_bot/tree/master/src/object_picker)

1. Run `roslaunch ownage_bot.launch manual:=true`
2. For actions that have no targets, call `rosservice call /action_provider/service_left "{action: 'action_name'}"`
3. For actions with objects as targets, call `rosservice call /action_provider/service_left "{action: 'action_name', object: {id: object_id}}"`
4. For actions with locations as targets, call `rosservice call /action_provider/service_left "{action: 'action_name', location: {x: x, y: y, z: z}}"`

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

1. Run `roslaunch ownage_bot.launch simulation:=true`
2. Wait for the results to be printed in the terminal

## Architecture

### Nodes

OwnageBot is comprised by five different modules, each of which runs as a ROS node to provide a certain functionality:

* [`object_tracker`](https://github.com/OwnageBot/ownage_bot/tree/master/nodes/object_tracker.py) - tracks all ARuco QR-tagged objects in the workspace, determining their absolute position (relative to Baxter's base frame), color, and proximity to avatars.
* [`object_collector`](https://github.com/OwnageBot/ownage_bot/tree/master/nodes/object_collector.py) - contains the main autonomous loop, repeatedly calling services provided by `object_picker` in order to scan the workspace and collect any unowned objects (where ownership is determined by `object_classifier`).
* [`object_classifier`](https://github.com/OwnageBot/ownage_bot/tree/master/nodes/object_classifier.py) - receives labelled example data from `object_collector` whenever an object thought to be unowned is claimed by another agent, which is then used to train the classifier.
* [`object_tester`](https://github.com/OwnageBot/ownage_bot/tree/master/nodes/object_tester.py) - generates virtual objects, avatars and examples, in order to train and test the learning algorithm in `object_classifier`.
* [`action_provider`](https://github.com/OwnageBot/ownage_bot/tree/master/src/action_provider) - provides high-level arm control as ROS services by inheriting the ArmCtrl interface in [`human_robot_collaboration_lib`](https://github.com/ScazLab/human_robot_collaboration/tree/master/human_robot_collaboration_lib).

### Topics & Services

When using `ownage_bot.launch`, these topic and service names are contained with the namespace `/ownage_bot` unless otherwise specified.

* `new_object` (topic)
  * Type: `uint32` (unsigned int)
  * Publisher(s): `object_tracker`
  * Subscriber(s): `action_provider`
  * Whenever `object_tracker` sees an ID that it has never seen before, it publishes to this topic.
* `feedback` (topic)
  * Type: [`FeedbackMsg`](https://github.com/OwnageBot/ownage_bot/blob/master/msg/FeedbackMsg.msg)
  * Publisher(s): `object_collector`, `object_tester` (in simulation)
  * Subscriber(s): `object_classifier`
  * Whenever Baxter successfully claims an object for itself, or is stopped and told the owner of the object, `object_collector` sends feedback to `object_classifier` as training data, using this topic. The three pieces of information contained in the feedback data structure are the *time of interaction*, *owner label* (avatar id of owner, 0 if unowned), and the *object data* itself.
* `reset_classifier` (topic)
  * Type: Empty
  * Publisher(s): `object_tester` (in simulation)
  * Subscriber(s): `object_classifier`
  * `object_classifier` clears its history of past interactions whenever it receives this signal.
* `lookup_object` (service)
  * Type: [`LookupObject`](https://github.com/OwnageBot/ownage_bot/blob/master/srv/LookupObject.srv)
  * Server(s): `object_tracker`
  * Client(s): `action_provider`
  * Looks up an object by ID when a single object is needed from the tracker
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
* `/action_provider/service_left` (service)
  * Type: [`CallAction`](https://github.com/OwnageBot/ownage_bot/blob/master/srv/CallAction.srv)
  * Server(s): `action_provider`
  * Client(s): `object_collector`
  * Used to perform various high-level actions using Baxter's left arm. Takes the action name and a list of objects as input, returns whether the action was successful along with a response string (e.g. an error message). See above for a list of supported actions.
