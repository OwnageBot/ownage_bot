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

For testing and debugging the arm control services provided by `object_picker'

1. Run `roslaunch ownage_bot.launch is_manual:=true`
2. Call service: `rosservice call /action_provider/service_left "{action: 'action_name', objects: [list of object ids]}"`

### List of supported actions (left arm only)

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

For simulated training and testing of `object_classifier`

1. Run `roslaunch ownage_bot.launch is_simulation:=true`
2. Wait for the results to be printed in the terminal