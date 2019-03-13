## Ping Echo Sounder Nodelet
Based on: [bluerobotics/ping-python/brping/pingmessage.py](https://github.com/bluerobotics/ping-python/blob/master/brping/pingmessage.py)

Currently only working with simple distance messages (distance in meters and confidence measures in percentages).

## To Do
* Improve project design.
* Extend to other ping messages.
* Set frame id.

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
