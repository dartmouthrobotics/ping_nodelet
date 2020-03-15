## Ping Echo Sounder Nodelet

ROS nodelet for the Blue Robotics Ping1D echosounder. (May work with Ping360 with minor modifications.)

[Dartmouth Reality and Robotics Lab](http://rlab.cs.dartmouth.edu/home/)

Authors: [Monika Roznere](http://monikaroznere.com/) and [Alberto Quattrini Li](https://sites.google.com/view/albertoq)

Based on: [bluerobotics/ping-cpp](https://github.com/bluerobotics/ping-cpp)

Currently publishes ROS topic messages based on the [Ping1D](https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/) (1211) distance_simple message. One may change the type of request and received message via `/include/ping_nodelet/ping_depth.h` file after the comments with `MESSAGE`. However, if one changes the message type, then the ROS topic message must also be changed in `/msg/Ping.msg`

## To Do
* Extend to other ping messages via xml file or other means.
* Add more parameters to the ROS ping echo sounder message to match with `sensor_msgs/Range` message type:
  * radiation_type: 0   (sonar)
  * field_of_view: 0.0  (or say 30 degrees)
  * min_range: 0.5      (meters)
  * max_range: 30.0     (meters)
* Create services for setting parameters, then update `pingmessage.py` to use ROS.
* Check if `start_ping.sh` is necessary for current firmware.


## Raspberry Pi Setup
* Install [ping-python](https://github.com/bluerobotics/ping-python)
* Move `start_ping.sh` in `~/companion/scripts`
  * Run:
    ``` console
    $ chmod +x start_ping.sh
    ```
* In `~/companion/.companion.rc` add this line:
``` console
$ sudo -H -u pi screen -dm -S ping $COMPANION_DIR/scripts/start_ping.sh
```

## Getting Started
This package has been tested on Ubuntu 16.04.

Set up your catkin workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace
```

## Install
Clone repository:
```
git clone https://github.com/dartmouthrobotics/underwater_color_enhance.git
```

## Build
To build:
```
catkin_make
source devel/setup.bash
```

## Configuration
To change parameters, edit the end of the `pingmessage.py` file, then run it:
```
python pingmessage.py
```

## Run
```
roslaunch ping_nodelet ping_echo_sounder.launch
```

Ping echo sounder data will be published to the `/ping_nodelet/ping` rostopic.
