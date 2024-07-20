# sample-contact-walking

## Useful commands
Enter the docker container: 
```
docker compose -f docker/docker-compose.yml run --build sample-walking
```

Build all the packages:
```
colcon build --symlink-install --parallel-workers $(nproc)
```

Source the package:
```
source install/setup.bash
```

Launch the stack:
```
obk-launch config_file_path=<file_path>.yaml device_name=onboard
```
