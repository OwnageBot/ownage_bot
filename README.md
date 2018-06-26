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

* `list <database>`: Lists available actions, tracked objects, learned rules, etc.
  * `list objects [simulated] <fields>...`: Lists objects as perceived by the tracker
    * If `simulated` is present, list all objects in the simulated environment, including those not tracked
    * Lists all specified fields, defaults to listing id, color, position and ownership
  * `list agents [simulated]`: Lists all agents and their names
    * If `simulated` is present, list all agents in the simulated environment, including those not tracked
  * `list predicates`: List names of all available predicates
  * `list rules`: List all currently active rules
  * `list actions`: Lists all actions that the robot can take
  * `list tasks`: List all higher-level tasks
* `reset <database>`: Resets the specified database
  * `reset perms`: Resets the permission database
  * `reset rules`: Resets the active rule database
  * `reset claims`: Resets the database of ownership claims
  * `reset objects`: Resets the database of tracked objects
  * `reset agents`: Resets the database of tracked agents
  * `reset simulation`: Resets and regenerates the simulated environment
  * `reset all`: Resets all of the above
* `(freeze|unfreeze) <database>`: Freezes changes to databases
  * `freeze perms`: Freezes the permission database (default unfrozen)
  * `freeze rules`: Freezes the rules database (default unfrozen) 
* `(disable|enable) <function>`: Disables certain learning capabilities
  * `disable inference`: Disables rule-based inference of ownership
  * `disable extrapolate`: Disables percept-based prediction of ownership
* `i am <agent>`: Make `<agent>` the current user and update the agent database accordingly
* `<action>`: Calls the corresponding action
* `<task>`: Calls the corresponding task
* `ownedBy <oid> <aid>`: Claim that object <oid> is owned by <aid>
* `(forbid|allow) <action> on <oid>`: Give object-specific permission for `<action>` on object `<oid>`
* `(forbid|allow) <action> if <predicate> <args> [and] ...)`: Give rule forbidding or allowing a certain action under the specified conditions
* `?<predicate> [pre-args] ? [post-args]`: Query for the arguments in the `?` slot and their corresponding truth values

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

## Architecture

### Nodes

OwnageBot is comprised of many different ROS nodes, each providing a certain functionality. They are roughly organized into similar functions below

#### Tracking and perception

*`agent_tracker` tracks the properties of all agents encountered as well as the identity of the current user
*`object_tracker` contains the abstract ObjectTracker class for tracking objects
*`aruco_tracker` inherits from `object_tracker` to implement object tracking through ArUco tags
*`endpoint_tracker` inherits from `object_tracker` to implement tracking of objects gripped by an endpoint manipulator
*`ownership_tracker` tracks and updates the ownership probabilities of each object
*`baxter_tracker` inherits from the above three nodes to combine their functionality for real-world object tracking
*`simulated_tracker` inherits from ownership_tracker and implements zero-noise tracking in a simulated environment

#### Rule and task management

*`rule_manager` manages and updates the rules learned through interaction with the environment
*`task_manager` carries out assigned actions and tasks, queuing them as necessary and checking if they are forbidden by the rules
*`rule_instructor` can be used to automatically train and evaluate the rule learning and ownership prediction capabilities

#### Simulation and visualization

*`world_simulator` generates and stores a simulated environment, from which `simulated tracker` gets data
*`world display` shows all currently tracked objects and their x-y locations in a 2D graphical display

#### Input/output

*`dialog_manager` handles both text and speech input, relaying the appropriate messages to and from other nodes and generating the responses
*`command_prompt` provides a command prompt for text input and output via curses
*`screen_manager` displays the camera feed and other relevant information on the Baxter screen
*`speech_processor` handles speech recognition and synthesis for (quasi-)natural dialog