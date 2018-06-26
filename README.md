# OwnageBot

A robotic system that learns the rules and relations of ownership based on interaction with objects and agents in its environment. Capable of ownership prediction through perceptual heuristics, ownership inference through Bayesian logic, and norm learning through incremental rule induction.

Developed primarily at the [Yale Social Robotics Lab](https://scazlab.yale.edu/).

## Prerequisites

If marked with an asterisk `*`, prerequisites are necessary even for a simulation-only compile.

### Python prerequisites
* [`scikit-learn`](http://scikit-learn.org/stable/index.html)*: for ownership prediction via logistic regression
* [`numpy`](http://www.numpy.org/)*: for inference related math operations
* [`PyAudio`](http://people.csail.mit.edu/hubert/pyaudio/): for accessing audio input
* [`pocketsphinx'](https://github.com/cmusphinx/pocketsphinx-python): for speech-to-text transcription

### ROS prerequisites
* [`aruco_ros`](https://github.com/ScazLab/aruco_ros): for recognition and tracking of QR codes
* [`cv_bridge`](http://wiki.ros.org/cv_bridge): for computer vision via OpenCV
* [`svox_tts`](https://github.com/ScazLab/svox_tts): for text-to-speech synthesis through SVOX
* [`human_robot_collaboration_lib`](https://github.com/ScazLab/human_robot_collaboration_lib): for arm control

## Compilation

We follow [`human_robot_collaboration`](https://github.com/ScazLab/human_robot_collaboration) in using `catkin_tools`.

### Full compile

1. Make sure you're on the correct branch/version of both `human_robot_collaboration_lib` and `aruco_ros`
2. Compile `human_robot_collaboration_lib`, `aruco_ros` and 'svox_tts' if necessary
3. Compile `ownage_bot`: `catkin build ownage_bot`

### Simulation-only compile

If you have a ROS installation but don't have `human_robot_collaboration`, `aruco_ros` or 'svox_tts', you can still compile and run the learning algorithm in
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

#### List of supported commands

* `list`: Lists available actions, tracked objects, learned rules, etc.
  * `list objects [simulated] <fields>...`: Lists all objects and their fields
    * If the `simulated` keyword is present, list all objects and their true (simulated) values, otherwise list objects as perceived by the tracker
    * Lists all specified fields, defaults to listing id, color, position and ownership
  * `list agents [simulated]': Lists all agents and their names
    * If the `simulated` keyword is present, list all agents in the simulated environment, including those not currently tracked
  * `list predicates`: List names of all available predicates
  * `list rules`: List all currently active rules
  * `list actions`: Lists all actions that the robot can take
  * `list tasks`: List all higher-level tasks
* `reset`: Resets the specified database
  * `reset perms`: Resets the permission database
  * `reset rules`: Resets the active rule database
  * `reset claims`: Resets the database of ownership claims
  * `reset objects`: Resets the database of tracked objects
  * `reset agents`: Resets the database of tracked agents
  * `reset simulation`: Resets and regenerates the simulated environment
  * `reset all`: Resets all of the above

### Manual arm control

For testing and debugging the arm control service provided by [`action_provider`](https://github.com/OwnageBot/ownage_bot/tree/master/src/action_provider)

1. Run `roslaunch ownage_bot.launch manual:=true`
2. For actions that have no targets, call `rosservice call /action_provider/service_left "{action: 'action_name'}"`
3. For actions with objects as targets, call `rosservice call /action_provider/service_left "{action: 'action_name', object: {id: object_id}}"`
4. For actions with locations as targets, call `rosservice call /action_provider/service_left "{action: 'action_name', location: {x: x, y: y, z: z}}"`

#### List of supported actions (left arm only)

* `goHome`: moves the arm to a position above its home area
* `release`: turns off vacuum gripper at current height and location
* `moveTo`: moves arm to specified location in 3D space (requires location)
* `find`: moves arm over the location of specified object (requires object ID)
* `pickUp`: picks up an object with a vacuum gripper (requires object ID)
* `putDown`: puts down an object gently at current x-y location
* `replace`: replaces object in last pick-up location
* `wait`: waits 3 seconds, can be interrupted by feedback from cuff button
