# plato_hololens

First, you should install project and run gazebo simulation

Put nodes from this repository to src/plato then run `catkin build octolens` `catkin build octomesh`

Octomap installation
`sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-rosbridge_suite`

Rosbrige installation
`sudo apt install ros-melodic-rosbridge-suite ros-melodic-rosbridge-server`


Octomap node execution
`cd ~/plato_ws/src/plato/octolens/launch`
`roslaunch octolens octomap.launch`

`roslaunch rosbridge_server rosbridge_websocket.launch`

Compressed markers(columns) here

`/plato_ws/src/plato/octolens/scripts$ python compressed_markers.py`

To publish 3d map
`for i in {1..10000}; do rosrun octomesh mesh; done`



In order to dump bt file (3d map)
`rosrun octomap_server octomap_saver -f my_octomap.bt`
