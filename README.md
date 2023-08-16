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

Run the image, using the host computer network to make it easier for your nodes to communicate (otherwise, you’ll have to individually open ports for every new node, I think):

`docker run -it –net=host ros_bridge_image /bin/bash`

Do not source any workspaces in this container for now, or the bridge will get confused.  

TODO: We can communicate with other Docker containers running ROS2 nodes and using the host network but not yet with the host computer.  This will require further investigation.

Open additional terminals to interact with the container:

`docker ps`

Note the container name.

`docker exec -it <container_name> /bin/bash`

Source workspaces as needed to have at least two ROS1 terminals and one more ROS2 terminal for the listener/talker example:

`source /opt/ros/noetic/setup.bash`

`. ros2_humble/install/local_setup.bash`

Run roscore in a noetic terminal.  You should see that the ros master is running on port 11311, but if not, modify the steps below based on what your system displays.

In the first terminal you opened for the container, run:

```
export ROS_MASTER_URI=http://<your_system>:11311
ros2 run ros1_bridge dynamic_bridge
```

If the ros2 commands aren't found, try:

`. /ros2_humble/install/local_setup.bash`

and run the bridge again.

You’ll see some error messages about topics that aren’t successfully bridging.  Unless these are topics you need, you can ignore them.  The rest of the bridge is working.

To test basic connectivity:

In a humble terminal:

`ros2 run demo_nodes_py talker`

In a noetic terminal:

`rosrun roscpp_tutorials listener`

Swap the listener/ talker and re-run to confirm bi-directional connectivity.

To test communication with a ROS2 application running in a separate container (I'm using the humble branch from the dockwater repo), start that container using the host computer network:

`docker run -it –net=host dockwater:humble /bin/bash`

Source your ROS2 installation and try running the talker and/or listener from the new container.

Once this is all working you can try to run nodes either directly on your system or in another container that is running on the host network.  This isn't working in all cases yet.
