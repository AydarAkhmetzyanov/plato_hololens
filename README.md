# plato_hololens

http://wiki.ros.org/octomap_server

First, you should install project and run gazebo simulation

Dependency installation
`sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-rosbridge_suite`

Octomap node execution
`cd ~/plato_ws/src/plato/octolens/launch`
`roslaunch octolens octomap.launch`



`roslaunch rosbridge_server rosbridge_websocket.launch`

To publish 3d map
`
for i in {1..10000}; do rosrun octomesh mesh; done
`



In order to dump bt file (3d map)
`rosrun octomap_server octomap_saver -f my_octomap.bt`
