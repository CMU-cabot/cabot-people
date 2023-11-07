FROM cmucal/cabot_people:ros2-latest


RUN mkdir src
COPY track_people_py src/track_people_py
COPY track_people_msgs src/track_people_msgs
COPY track_people_cpp src/track_people_cpp
COPY cabot_people src/cabot_people

RUN /ros_entrypoint.sh colcon build
