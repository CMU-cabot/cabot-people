# CaBot People repo

|Package|Description|
|---|---|
|[cabot_people](../cabot_people)|launch file and script for launching people tracking nodes|
|[track_people_cpp](../track_people_cpp)|detect and track people (cpp implementation) for improved performance|
|[track_people_msg](../track_people_msg)|msgs for track_people packages|
|[track_people_py](../track_people_py)|detect and track people|


## Docker environment for development build

```
docker compose build
docker compose run --rm ros2 colcon build
```

## Test

TBD

# License

[MIT License](LICENSE)
