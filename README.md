# CaBot People repo

|Package|Description|
|---|---|
|[cabot_people](../cabot_people)|launch file and script for launching people tracking nodes|
|[track_people_cpp](../track_people_cpp)|detect and track people (cpp implementation) for improved performance|
|[track_people_msg](../track_people_msg)|msgs for track_people packages|
|[track_people_py](../track_people_py)|detect and track people|

## Test

### Preparation

- run the script to download dependencies

```
./setup-dependency.sh
```

- assume you have docker (Nvidia docker) and docker compose
- make sure you have a PC with a NVIDIA GPU, or a Jeston (Xavier, Orin, Xavier NX)

- if you pull the latest docker images from docker hub, run the following command

```
docker compose --profile build pull
```

- if you build docker image, run the script to build image

```
./bake-docker.sh -i         # run docker image build for your platform
```

- if you run in development mode, run the script to build workspaces

```
./build-workspace.sh        # run workspace build
./build-workspace.sh -d     # run workspace debug build (symlink-install)
```

### Bringup Realsense(s), detection, and tracking

- connect realsense(s) to your PC
- edit `.env` file to specify Relasense serial numbers (required if you use multiple)
```
CABOT_REALSENSE_SERIAL_1=
CABOT_REALSENSE_SERIAL_2=
CABOT_REALSENSE_SERIAL_3=
```
- run one of the following script after the build

```
docker compose -f docker-compose-test-rs3.yaml --profile prod up rs1-prod track-prod           # 1 Realsense in production mode
docker compose -f docker-compose-test-rs3.yaml --profile prod up rs1-prod rs2-prod track-prod  # 2 Realsenses in production mode
docker compose -f docker-compose-test-rs3.yaml --profile prod up                               # 3 Realsenses in production mode

docker compose -f docker-compose-test-rs3.yaml --profile dev up rs1-dev track-dev           # 1 Realsense in development mode
docker compose -f docker-compose-test-rs3.yaml --profile dev up rs1-dev rs2-dev track-dev   # 2 Realsenses in development mode
docker compose -f docker-compose-test-rs3.yaml --profile dev up                             # 3 Realsenses in development mode
```

```
docker compose -f docker-compose-test-rs3-framos.yaml --profile prod up rs1-framos-camera-prod rs1-framos-detection-prod track-framos-prod                                                      # 1 FRAMOS in production mode
docker compose -f docker-compose-test-rs3-framos.yaml --profile prod up rs1-framos-camera-prod rs1-framos-detection-prod rs2-framos-camera-prod rs2-framos-detection-prod track-framos-prod     # 2 FRAMOSes in production mode
docker compose -f docker-compose-test-rs3-framos.yaml --profile prod up                                                                                                                         # 3 FRAMOSes in production mode

docker compose -f docker-compose-test-rs3-framos.yaml --profile dev up rs1-framos-camera-dev rs1-framos-detection-dev track-framos-dev                                                  # 1 FRAMOS in development mode
docker compose -f docker-compose-test-rs3-framos.yaml --profile dev up rs1-framos-camera-dev rs1-framos-detection-dev rs2-framos-camera-dev rs2-framos-detection-dev track-framos-dev   # 2 FRAMOSes in development mode
docker compose -f docker-compose-test-rs3-framos.yaml --profile dev up                                                                                                                  # 3 FRAMOSes in development mode
```

### Check `/people` topic

- check if `/people` topic is published and recognize someone in front of the camera
- check if `/people` topic is published at 10-15 hz per camera, if you have two cameras it publishes at 20-30 hz.

```
docker exec -it $(docker ps -f name=cabot-people-rs1 -q) bash -c "source install/setup.bash && ros2 topic echo /people"
docker exec -it $(docker ps -f name=cabot-people-rs1 -q) bash -c "source install/setup.bash && ros2 topic hz /people"
```

# License

[MIT License](LICENSE)
