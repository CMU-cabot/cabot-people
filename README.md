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
- run one of the following the script to build image and workspaces

```
./build-docker.sh -p -i -w              # build image and workspaces for RealSense
./build-docker.sh -p -i -w -c framos    # build image and workspaces for FRAMOS
./build-docker.sh -p -i -w -c all       # build image and workspaces for RealSense and FRAMOS
```

- run the script to prepare people detection models after you build image

```
./setup-model.sh
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
docker compose -f docker-compose-test-rs3.yaml up rs1 track                   # 1 Realsense on PC
docker compose -f docker-compose-test-rs3.yaml up rs1 rs2 track               # 2 Realsenses on PC
docker compose -f docker-compose-test-rs3.yaml up                             # 3 Realsenses on PC
docker compose -f docker-compose-jetson-test-rs3.yaml up rs1 track            # 1 Realsense on Jetson
docker compose -f docker-compose-jetson-test-rs3.yaml up rs1 rs2 track        # 2 Realsenses on Jetson
docker compose -f docker-compose-jetson-test-rs3.yaml up                      # 3 Realsenses on Jetson
```

```
docker compose -f docker-compose-test-rs3-framos.yaml up rs1-framos-camera rs1-framos-detection track-framos                                                # 1 FRAMOS on PC
docker compose -f docker-compose-test-rs3-framos.yaml up rs1-framos-camera rs1-framos-detection rs2-framos-camera rs2-framos-detection track-framos         # 2 FRAMOSes on PC
docker compose -f docker-compose-test-rs3-framos.yaml up                                                                                                    # 3 FRAMOSes on PC
docker compose -f docker-compose-jetson-test-rs3-framos.yaml up rs1-framos-camera rs1-framos-detection track-framos                                         # 1 FRAMOS on Jetson
docker compose -f docker-compose-jetson-test-rs3-framos.yaml up rs1-framos-camera rs1-framos-detection rs2-framos-camera rs2-framos-detection track-framos  # 2 FRAMOSes on Jetson
docker compose -f docker-compose-jetson-test-rs3-framos.yaml up                                                                                             # 3 FRAMOSes on Jetson
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
