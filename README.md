# Docker container to enable noetic <-> humble communication on Ubuntu Focal

Inspired by and loosely based on:

https://github.com/contradict/ros-humble-ros1-bridge


See also various references:

https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html?highlight=bridge

https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html


# How to get started
Grab this repo.

Build the image (I’m tagging it ros_bridge_image here):

`docker build -t ros_bridge_image .`


Run the image:

`docker run -it ros_bridge_image /bin/bash`

Do not source any workspaces in this terminal for now, or the bridge will get confused.  


Open additional terminals to interact with the container:

`docker ps`


- note the container name
  
`docker exec -it <container_name> /bin/bash`

Source workspaces as needed to have at least two ROS1 terminals and one more ROS2 terminal for the listener/talker example:
```
source /opt/ros/noetic/setup.bash
. ros2_humble/install/local_setup.bash
```


Run roscore in a noetic terminal.  You should see that the ros master is running on port 11311, but if not, modify the steps below based on what your system displays.

In the first terminal you opened for the container, run:
```
export ROS_MASTER_URI=http://<your_system>:11311
ros2 run ros1_bridge dynamic_bridge
```

If the ros2 commands aren't found, try:

`. /ros2_humble/install/local_setup.bash`

and run the bridge again.


### To test basic connectivity:

In a humble terminal:

`ros2 run demo_nodes_py talker`

In a noetic terminal:

`rosrun roscpp_tutorials listener`

Swap the listener/ talker and re-run to confirm bi-directional connectivity.



### To test the ability to bridge a ParamVec message:

In a humble terminal:

`ros2 topic pub /tester ros_gz_interfaces/ParamVec "{header: {stamp: now, frame_id: 'map'}, params: []}"`

In a noetic terminal:

`rostopic echo /tester`

This might work; if not, open some more terminals and try to wake the bridge up:
```
source/opt/ros/noetic/setup.bash
rostopic pub /tester vrx_bridge_msgs/ParamVec "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
params:
- name: ''
  value:
    type: 0
    bool_value: false
    integer_value: 0
    double_value: 0.0
    string_value: ''
    byte_array_value: [0]
    bool_array_value: [false]
    integer_array_value: [0]
    string_array_value: ['']"
```
```
source /opt/ros/noetic/setup.bash
rostopic list
```

You should see that the bridge is created for the ParamVec topic and that the "map" frame message from ROS2 is echoed in the ROS1 terminal. 

### To test with [VRX](https://github.com/osrf/vrx):

Copy the "basic_node.py" file into your running docker container.  

In a ROS1 terminal, run the node: 

`python3 basic_node.py`

In another container or on your computer, run VRX:

`ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task.sdf`

When the stationkeeping task reaches the "running" state, you should see the WAM-V move forward briefly and then stop.  You can confirm this by echoing the /wamv/thrusters/left/thrust command in a ROS2 terminal.

Running the perception task should produce no thrust commands.  Running any other task should cause the WAM-V to move forward without stopping.



Useful information about this example and lessons learned for customizing your own bridge: 
* This image has a local, stripped down package that copies the necessary parts of ros-gz-interfaces to provide the ParamVec ROS2 message, so that we don't have to include all of Gazebo in the Docker image.  This may not be the right final solution but it works for the proof-of-concept.
* A recent change to the bridge package broke some of the functionality needed to compile the bridge with the Parameter message type enabled.  bool vectors are the likely culprit; see [this issue](https://github.com/ros2/ros1_bridge/issues/393) for details.  Because of this we are using a specific older commit when cloning the bridge software.
* Only packages ending in _msgs (ROS1 and ROS2) or _interfaces (ROS2) are found by the bridge during compilation.  You can't override this behavior by explicitly listing other packages in the .yaml mapping file.
* ROS1 messages are installed to /opt/ros/noetic and ROS2 messages are installed to /ros2_humble/install.  This is useful for minimizing the overlays/ underlays that you have to source but is not required.
* The editor installed in this image is nano.  
* Everything seems to be working as of 22 August 2023.  If you run into compilation problems, try reverting back to earlier commits from around that date.


