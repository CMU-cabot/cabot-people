# CaBot People repo

|Package|Description|
|---|---|
|[cabot_people](../cabot_people)|launch file and script for launching people tracking nodes|
|[track_people_cpp](../track_people_cpp)|detect and track people (cpp implementation) for improved performance|
|[track_people_msg](../track_people_msg)|msgs for track_people packages|
|[track_people_py](../track_people_py)|detect and track people|

## Prepare yolov4 model

- run the script to download yolov4 model

```
./setup-model.sh
```

## Docker environment for development build (using prebuild image)

- make sure you have a PC with a NVIDIA GPU
- docker and nvidia-docker is installed and configured

```
docker compose build ros2
docker compose run --rm ros2 bash
colcon build
```

### Test

- connect a realsense to your PC
- run the following script after the build

```
/launch.sh -r -t 0 -D -K
```

- check if `/people` topic is published and recognize someone in front of the camera

```
docker exec -it $(docker ps -f name=cabot-people-ros2 -q) bash
source install/setup.bash
ros2 topic echo /people
```

# License

[MIT License](LICENSE)
