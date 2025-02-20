# sample-contact-walking

## TODO for the docker:
- Add Proxqp to the docker
- Make obelisk pull to the uniree-interface branch
- Add ping to the docker
- Add the optitrack library
- Add lshw (apt-get install -y lshw)
- Add python3-pyqt5 (apt-get install python3-pyqt5)

## Useful commands
Setup:
```
bash setup.sh
```

Enter the docker container: 
```
docker compose -f docker/docker-compose.yml run --build sample-walking
```

Update library path for hpipm:
```
export LD_LIBRARY_PATH=/opt/blasfeo/lib:/opt/hpipm/lib:$LD_LIBRARY_PATH
```

Build and activate Obelisk:
```
obk
```

Build the messages pacakge first:
```
colcon build --symlink-install --packages-select sample_contact_msgs
```

Build all the packages:
```
colcon build --symlink-install --parallel-workers $(nproc)
```

Build packages with verbose output:
```
colcon build --symlink-install --parallel-workers $(nproc) --event-handlers console_direct+
```

Build packages in debug mode:
```
colcon build --symlink-install --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Source the package:
```
source install/setup.bash
```

Set logging dir:
```
export ROS_HOME=~/sample-contact-walking
```
## Launch the Achilles stack:
```
obk-launch config_file_path=${SAMPLE_WALKING_ROOT}/sample_contact_walking/configs/achilles_sim_config.yaml device_name=onboard auto_start=configure bag=false
```

Wait for the viz software to connect then run in a seperate terminal:
```
obk-activate achilles_sim
```

## Launch the Go2 stack:
```
obk-launch config_file_path=${SAMPLE_WALKING_ROOT}/sample_contact_walking/configs/go2_sim_config.yaml device_name=onboard auto_start=configure bag=false
```

Wait for the viz software to connect then run in a seperate terminal:
```
obk-activate go2_sim
```

## Launch the G1 stack:
```
obk-launch config_file_path=${SAMPLE_WALKING_ROOT}/sample_contact_walking/configs/g1_sim_config.yaml device_name=onboard auto_start=configure bag=false
```
 
Wait for the viz software to connect then run in a seperate terminal:
```
obk-activate g1_sim
```

If you have issues with others on the ROS network then set `ROS_LOCALHOST_ONLY`.

## Connecting the joystick
### USB
Can verify that the the controller connects via
```
sudo apt-get update
sudo apt-get install evtest
sudo evtest /dev/input/eventX
```
where you replace eventX with the correct number. You can see these by looking at `/dev/input/`.

Then you may need to change the permissions for the joystick:
```
sudo chmod 666 /dev/input/eventX
```
event24 seems to be consistent for the xbox remote for my machine.

Can run 
```
ros2 run joy joy_enumerate_devices
``` 
to see what devices are found.


## Random notes
<!-- ## Python deps (not yet added to the docker)
- Scipy (pip)
- Mujoco -> Comes with obelisk
- OSQP (pip) -->

- Remember that the topics in the contact planner need to updated in the source code until Obelisk is updated
- If all the ros commands are hanging try:
```
ps aux | grep ros2
```
```
pkill -9 -f ros2
```
To see then kill all the poosible processes

## Running the Unitree Interface
- Need to set the local enivornment variable: OBELISK_BUILD_UNITREE=true
- Need to change the ROS_DOMAIN_ID to be different (going to 5 works)
```
export ROS_DOMAIN_ID=5
```
Be sure to change the `ROS_DOMAIN_ID` in all terminals!
```
export OBELISK_BUILD_UNITREE=true
```

### Connecting to the robot
- Make sure you can run `ping 192.168.123.161` and see the robot
- Run the basic G1 obelisk example
```
obk-launch config_file_path=g1_cpp.yaml device_name=onboard bag=false
```
Make sure the joints move and that you can cycle through them.

### Launch the G1 Hardware stack:
```
obk-launch config_file_path=${SAMPLE_WALKING_ROOT}/sample_contact_walking/configs/g1_hardware_config.yaml device_name=onboard auto_start=configure bag=false
```

Wait for the viz software to connect then run in a seperate terminal:
```
obk-activate g1_hardware
```

### Seting up the Mocap
We use https://github.com/L2S-lab/natnet_ros2 which I hope to eventually add into obelisk for automatic installation.
Make sure this is installed. For now I am installing it in `~/sample-contact-walking`

Note that you should be connected to the network with the optitrack computer via ethernet.
You should set the `Server IP` as the opti track computer's IP (normally `192.168.1.2`).
`Client IP` should be your IP.

Launch with 
```
ros2 launch natnet_ros2 gui_natnet_ros2.launch.py
```
I had to run
```
mkdir -p /tmp/runtime-$USER
chmod 700 /tmp/runtime-$USER
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
```
To make a temp directory that I had permissions to write to first otherwise the nat net driver complains.


## Torque Testing
Run the stack
```
obk-launch config_file_path=${SAMPLE_WALKING_ROOT}/sample_contact_walking/configs/basic_torque_config.yaml device_name=onboard auto_start=configure bag=false
```
```
obk-activate g1_basic_ctrl_hardware
```


## On the Robot
```
ros2 launch perception_node perception_launch_tracking_only.py
```