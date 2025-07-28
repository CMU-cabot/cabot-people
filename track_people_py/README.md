# track_people_py package

detect and track people

## scripts/detect_darknet_opencv_people.py

detect people using OpenCV implementation of darknet in an RGB image and estimate position from a depth image

## scripts/detect_mmdet_people.py

detect people using MMDetection implementation in an RGB image and estimate position from a depth image

### publish
- **/track_people_py/detected_boxes**: detected people location without track id

### subscribe
topic name can be changed by the parameter

- **/camera/color/camera_info**: to get camera size
- **/camera/color/image_raw**: RGB image
- **/camera/aligned_depth_to_color/image_raw**: Depth image

### service
- **enable_detect_people**: to control whether detecting people


## scripts/track_sort_3d_people.py

find the corresponding person in detected people to assign track id

### publish
- **/track_people_py/combined_detected_boxes**: detected people from all cameras
- **/people**: tracked people

### subscribe
- **/track_people_py/detected_boxes**: detected people

