<meta name="google-site-verification" content="fhADYjOr6bTO7B-pFZSUWxqBDKt6yfWbMZNDKJic0Js" />

## Ping Echo Sounder Nodelet
Based on: [bluerobotics/ping-python/brping/pingmessage.py](https://github.com/bluerobotics/ping-python/blob/master/brping/pingmessage.py)

Currently only working with simple distance messages (distance in meters and confidence measures in percentages).

## To Do
* Improve project design.
* Extend to other ping messages.
* Set frame id.
* Add more parameters to the ros ping echo sounder message to match with `sensor_msgs/Range` message type:
  * radiation_type: 0   (sonar)
  * field_of_view: 0.0  (or say 30 degrees)
  * min_range: 0.5      (meters)
  * max_range: 30.0     (meters)
* Fix bug as said in ping_depth.h about initializing PingParser 


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

## Run
``` console
$ roslaunch ping_nodelet ping_echo_sounder.launch
```

Ping echo sounder data will be published to the `/ping_nodelet/ping` rostopic.
