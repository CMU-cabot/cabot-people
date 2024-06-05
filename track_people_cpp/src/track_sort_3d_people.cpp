
#include "track_sort_3d_people.hpp"

namespace track_people_cpp
{
TrackerSort3D::TrackerSort3D(
  double iou_threshold=0.01, double iou_circle_size=0.5,
  double minimum_valid_track_duration=0.3,
  double duration_inactive_to_remove=2.0,
  int n_colors=100)
  : iou_threshold_(iou_threshold),
    iou_circle_size_(iou_circle_size),
    minimum_valid_track_duration_(minimum_valid_track_duration),
    duration_inactive_to_remove_(duration_inactive_to_remove),
    n_colors_(n_colors)
{
  /* 
    Initialization
    
    iou_threshold : minimum IOU threshold to keep track
    iou_circle_size : radius of circle in bird-eye view to calculate IOU
    minimum_valid_track_duration : minimum duration to consider track is valid
    duration_inactive_to_remove : duration for an inactive detection to be removed
    n_colors : number of colors to assign to each track
  */

  // parameters for Kalman Filter
  kf_time_step_ = 1.0/1.0;
  kf_sigma_proc_ = 10.0;
  kf_sigma_meas_ = 10.0;
  sigma_l_ = 0.0;
  sigma_h_ = 0.5;

  // set data keepers
  record_tracker_ = std::vector<int>();
  box_active_ = std::vector<int>();
  kf_active_ = std::vector<int>();
  record_tracker_archive_ = std::vector<int>();

  // set parameters
  iou_threshold_ = iou_threshold;
  iou_circle_size_ = iou_circle_size;
  minimum_valid_track_duration_ = minimum_valid_track_duration;
  duration_inactive_to_remove_ = duration_inactive_to_remove;
  n_colors_ = n_colors;
  /* self.list_colors = plt.cm.hsv(np.linspace(0, 1, n_colors)).tolist()  # list of colors to assign to each track for visualization
      np.random.shuffle(self.list_colors)  # shuffle colors */

  // counter of tracks
  tracker_count_ = 0;
}

void TrackerSort3D::track(
  int* prev_exist, int* person_id, int* person_color, int* trackeduration, 
  int now, int bboxes, int center_pos_list, int frame_id,
  int counter_penalty=1, bool drop_inactive_feature=true)
{
  /* WIP */
}

void TrackerSort3D::get_track_id(int id_track)
{
  /* WIP */
}

void TrackerSort3D::get_active_tracks_at_frame(int id_frame)
{
  /* WIP */
}

AbsTrackPeople::AbsTrackPeople(
  char* node_name, rclcpp::NodeOptions options,
  char* device, int minimum_valid_track_duration)
  : Node(node_name, options),
    minimum_valid_track_duration_(minimum_valid_track_duration),
    device_(device),
    frame_id_(0),
    prev_detect_time_sec_(0)
{
  /*
  detected_boxes_sub_ = create_subscription(TrackedBoxes, )
  tracked_boxes_pub_ = create_pub
  visualization_marker_array_pub_ = create_pub

  updater = Updater(self)

  target_fps = self.declare_parameter('target_fps', 0.0).value
  diagnostic_name = self.declare_parameter('diagnostic_name', "PeopleTrack").value
  self.htd = HeaderlessTopicDiagnostic(diagnostic_name, self.updater,
                                        FrequencyStatusParam({'min': target_fps/3.0, 'max': target_fps}, 0.2, 2))
  */
}

void AbsTrackPeople::detected_boxes_cb(int detected_boxes_msg)
{
}

void AbsTrackPeople::preprocess_msg(int detected_boxes_msg)
{
  /*
  detect_results = []
  center_bird_eye_global_list = []
  for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
      detect_results.append([bbox.box.xmin, bbox.box.ymin, bbox.box.xmax, bbox.box.ymax])
      center_bird_eye_global_list.append([bbox.center3d.x, bbox.center3d.y, bbox.center3d.z])
  return np.array(detect_results), center_bird_eye_global_list
*/
}

void AbsTrackPeople::pub_result(
  int detected_boxes_msg, int id_list, int color_list, int tracked_duration)
{
/*
  # publish tracked boxes message
  tracked_boxes_msg = TrackedBoxes()
  tracked_boxes_msg.header = detected_boxes_msg.header
  tracked_boxes_msg.camera_id = detected_boxes_msg.camera_id
  tracked_boxes_msg.pose = detected_boxes_msg.pose
  for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
      if tracked_duration[idx_bbox] < self.minimum_valid_track_duration:
          continue
      tracked_box = TrackedBox()
      tracked_box.header = bbox.header
      tracked_box.track_id = id_list[idx_bbox]
      tracked_box.color = ColorRGBA(r=color_list[idx_bbox][0], g=color_list[idx_bbox][1], b=color_list[idx_bbox][2], a=0.0)
      tracked_box.box = bbox.box
      tracked_box.center3d = bbox.center3d
      tracked_boxes_msg.tracked_boxes.append(tracked_box)
  self.tracked_boxes_pub.publish(tracked_boxes_msg)

  self.get_logger().info("camera ID = " + detected_boxes_msg.camera_id + ", number of tracked people = " + str(len(tracked_boxes_msg.tracked_boxes)))
*/
}

void AbsTrackPeople::vis_result(
  int detected_boxes_msg, int id_list, int color_list, int tracked_duration)
{
/*
  # publish visualization marker array for rviz
  marker_array = MarkerArray()
  for idx_bbox, bbox in enumerate(detected_boxes_msg.tracked_boxes):
      if tracked_duration[idx_bbox] < self.minimum_valid_track_duration:
          continue
      marker = Marker()
      marker.header = bbox.header
      marker.ns = "track-people"
      marker.id = id_list[idx_bbox]
      marker.type = Marker.CUBE
      marker.action = Marker.ADD
      marker.lifetime = Duration(nanoseconds=500000000).to_msg()
      marker.scale.x = 0.5
      marker.scale.y = 0.5
      marker.scale.z = 0.2
      marker.pose.position = bbox.center3d
      marker.pose.orientation.x = 0.0
      marker.pose.orientation.y = 0.0
      marker.pose.orientation.z = 0.0
      marker.pose.orientation.w = 1.0
      marker.color.r = color_list[idx_bbox][0]
      marker.color.g = color_list[idx_bbox][1]
      marker.color.b = color_list[idx_bbox][2]
      marker.color.a = 1.0
      marker_array.markers.append(marker)
  self.visualization_marker_array_pub.publish(marker_array)
*/
}

TrackSort3dPeople::TrackSort3dPeople(rclcpp::NodeOptions options)
: device_("cuda"),
  // minimum valid duration should be always 0 for multi camera
  // minimum_valid_track_duration = rospy.Duration(0.3) # Minimum duration to consider track is valid
  minimum_valid_track_duration_(0), //Minimum duration to consider track is valid
  iou_threshold_(0.01), // IOU threshold to consider detection between frames as same person
  iou_circle_size_(1.0), // radius of circle in bird-eye view to calculate IOU
  duration_inactive_to_remove_(2.0), //duration (seconds) for a track to be inactive before removal
  AbsTrackPeople("track_sort_3d_people_node", options, device_, minimum_valid_track_duration_)
{
  // set tracker
  tracker = std::shared_ptr<TrackerSort3D>(new TrackerSort3D(
    iou_threshold_, iou_circle_size_, minimum_valid_track_duration_,
    duration_inactive_to_remove_));

    combined_detected_boxes_pub_ = super->create_publisher<track_people_msgs::msg::TrackedBoxes>('people/combined_detected_boxes', 10);
  /*

  self.buffer = {}
  */

/*
  plt.ion()
  plt.show()
  rclpy.spin(track_people)
*/
}

void TrackSort3dPeople::detected_boxes_cb(int detected_boxes_msg)
{
/*
  self.htd.tick()
  self.get_logger().info("detected_boxes_cb")
  # check if tracker is initialized
  if not hasattr(self, 'tracker'):
      return

  now = self.get_clock().now()

  # To ignore cameras which stop by accidents, remove detecion results for cameras that are not updated longer than threshold to remove track
  delete_camera_ids = []
  for key in self.buffer:
      if (now - rclpy.time.Time.from_msg(self.buffer[key].header.stamp)) > self.tracker.duration_inactive_to_remove:
          delete_camera_ids.append(key)
  for key in delete_camera_ids:
      self.get_logger().info("delete buffer for the camera which is not updated, camera ID = " + str(key))
      del self.buffer[key]

  # 2022.01.12: remove time check for multiple detection
  # check if image is received in correct time order
  # cur_detect_time_sec = detected_boxes_msg.header.stamp.to_sec()
  # if cur_detect_time_sec<self.prev_detect_time_sec:
  #    return

  self.buffer[detected_boxes_msg.camera_id] = detected_boxes_msg

  combined_msg = None
  for key in self.buffer:
      msg = copy.deepcopy(self.buffer[key])
      if not combined_msg:
          combined_msg = msg
      else:
          combined_msg.tracked_boxes.extend(msg.tracked_boxes)
  combined_msg.header.stamp = now.to_msg()

  detect_results, center_bird_eye_global_list = self.preprocess_msg(combined_msg)

  self.combined_detected_boxes_pub.publish(combined_msg)

  try:
      _, id_list, color_list, tracked_duration = self.tracker.track(now, detect_results, center_bird_eye_global_list, self.frame_id)
  except Exception as e:
      self.get_logger().error(F"tracking error, {e}")
      return

  self.pub_result(combined_msg, id_list, color_list, tracked_duration)

  self.vis_result(combined_msg, id_list, color_list, tracked_duration)

  self.frame_id += 1
  # self.prev_detect_time_sec = cur_detect_time_sec
*/
}

void TrackSort3dPeople::receiveSignal(int signal_num, int frame)
{
/*
  print("Received:", signal_num)
  sys.exit(0)


  signal.signal(signal.SIGINT, receiveSignal)
*/
}

}  // namespace track_people_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(track_people_cpp::TrackSort3dPeople)
