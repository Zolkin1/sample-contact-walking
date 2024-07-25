# sample-contact-walking

## Useful commands
Setup:
```
bash setup.sh
```

Enter the docker container: 
```
docker compose -f docker/docker-compose.yml run --build sample-walking
```

Source base ROS:
```
source /opt/ros/humble/setup.bash
```

Build and activate Obelisk:
```
obk
```

Build all the packages:
```
colcon build --symlink-install --parallel-workers $(nproc)
```

Build packages with verbose output:
```
colcon build --symlink-install --parallel-workers $(nproc) --event-handlers console_direct+
```

Source the package:
```
source install/setup.bash
```

Set logging dir:
```
export ROS_HOME=~/sample-contact-walking
```

Launch the stack:
```
obk-launch config_file_path=${SAMPLE_WALKING_ROOT}/sample_contact_walking/configs/achilles_sim_config.yaml device_name=onboard auto_start=configure
```
Wait for the viz software to connect then run:
```
obk-activate achilles
```