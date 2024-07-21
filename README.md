# sample-contact-walking

## Useful commands
Enter the docker container: 
```
docker compose -f docker/docker-compose.yml run --build sample-walking
```

Source Obelisk:
```
source $OBELISK_ROOT/obelisk_ws/install/setup.bash
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

Launch the stack:
```
obk-launch config_file_path=<file_path>.yaml device_name=onboard
```
