
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
  std::vector<bool>* prev_exist, std::vector<int>* person_id,
  std::vector<std::vector<double>>* person_color,
  std::vector<double>* tracked_duration,
  const rclcpp::Time& now, const std::vector<std::vector<double>>& bboxes,
  const std::vector<std::vector<double>>& center_pos_list,
  int frame_id, int counter_penalty=1, bool drop_inactive_feature=true)
{
/*
  Performs tracking by comparing with previous detected people
    INPUT
  bboxes : n x 4+ matrix - each row is a bbox [x_tl, y_tl, x_br, y_br]
                            of each detection. Additional element (e.g.
                            detection score) may be included.
  center_bird_eye_list : n x 2 matrix - each row is a bbox [x, y]
                            in bird-eye view of each detection.
  frame_id : 1 x 1 scalar - id of the frame
  step_penalty : 1 x 1 scalar - amount to discount the active detection counter
  drop_inactive_feature : boolean - True to drop features of permanent inactive
                             detection to save space
    OUTPUT
  prev_exist : boolean n-vector indicating whether each of the
                person exists before
  person_id : int n-vector indicating id of each person
  person_color : visualization color of each each person
  tracked_duration : total tracked duration
*/

  std::vector<std::vector<double>> center_circle_list;
  for (auto center_pos : center_pos_list) {
    center_circle_list.push_back(std::vector<double>{center_pos[0], center_pos[1], iou_circle_size_});
  }

  // prepare output
  *prev_exist = std::vector<bool>(bboxes.size());
  *person_id = std::vector<int>(bboxes.size());
  *person_color = std::vector<std::vector<double>>(bboxes.size(), std::vector<double>());
  *tracked_duration = std::vector<double>(bboxes.size());

  if (bboxes.size() == 0 && box_active_.size() == 0) {
    // No new detection and no active tracks
    // Do nothing
    std::vector<int> det_to_add;
    std::vector<int> track_inactive;

    // predict box by Kalman Filter
    // WIP
    //for (int i=0; i<kf_active_.size(); i++) {
    //  kf_active_[i]["kf"].predict();
    //}
  }
/*
  elif (len(bboxes) == 0) and (len(self.box_active) > 0):
      # No new detection but has active tracks

      # no tracks to add
      det_to_add = []

      # set all active tracks to inactive
      track_inactive = list(self.box_active.keys())

      # predict box by Kalman Filter
      for id_track in self.kf_active.keys():
          self.kf_active[id_track]["kf"].predict()
  elif (len(bboxes) > 0) and (len(self.box_active) == 0):

      # If no active detection, add all of them
      det_to_add = np.arange(len(bboxes))

      # No track to be considered inactive
      track_inactive = []

      # predict box by Kalman Filter
      for id_track in self.kf_active.keys():
          self.kf_active[id_track]["kf"].predict()
  elif (len(bboxes) > 0) and (len(self.box_active) > 0):
      # If there are active detections, compared them to new detections
      # then decide to match or add as new tracks

      # predict circle by Kalman Filter
      kf_pred_circles = []
      for id_track in self.kf_active.keys():
          self.kf_active[id_track]["kf"].predict()

          kf_x = self.kf_active[id_track]["kf"].x[0, 0]
          kf_y = self.kf_active[id_track]["kf"].x[2, 0]
          kf_circle = [kf_x, kf_y, self.iou_circle_size]
          kf_pred_circles.append(kf_circle)

      # get all active tracks
      key_box_active = list(self.box_active.keys())

      # compute IOU
      iou = reid_utils_fn.compute_circle_pairwise_iou(center_circle_list, kf_pred_circles)

      # match by Hungarian
      row_ind, col_ind = linear_sum_assignment(1-iou)

      # get prev and current detection correspondence, then update existing tracks
      track_continue_current = []
      track_continue_prev = []
      for cur_idx, prev_idx in zip(row_ind, col_ind):
          if iou[cur_idx][prev_idx] > self.iou_threshold:
              track_continue_current.append(cur_idx)
              track_continue_prev.append(prev_idx)
      track_continue_prev = [key_box_active[i] for i in track_continue_prev]  # get id of still active tracks

      # Now we have a 1-1 correspondence of active tracks
      # between track id in track_continue_prev and detection in track_continue_current.
      # Add these infos to the record.
      for i, id_track in enumerate(track_continue_prev):
          circle_tmp = center_circle_list[track_continue_current[i]]
          self.record_tracker[id_track]["frame"][frame_id] = {}
          self.record_tracker[id_track]["frame"][frame_id]["bbox"] = bboxes[track_continue_current[i]]
          self.record_tracker[id_track]["expire"] = now + self.duration_inactive_to_remove
          self.box_active[id_track] = circle_tmp

          # update Kalman Filter
          kf_meas = [circle_tmp[0], circle_tmp[1]]
          self.kf_active[id_track]["kf"].update(np.asarray(kf_meas).reshape([len(kf_meas), 1]))
          self.kf_active[id_track]["missed"] = 0

          # set output
          prev_exist[track_continue_current[i]] = True
          person_id[track_continue_current[i]] = id_track
          person_color[track_continue_current[i]] = self.record_tracker[id_track]["color"]
          tracked_duration[track_continue_current[i]] = (now - self.record_tracker[id_track]["since"]).nanoseconds/1000000000

      # the rest of the tracks are new tracks to be add later
      det_to_add = np.setdiff1d(np.arange(len(bboxes)), track_continue_current)

      # get the list of tracks that have become inactive
      track_inactive = [x for x in key_box_active if x not in track_continue_prev]
  else:
      assert False, "Something is wrong here... All conditions shouldn't be wrong!"

  # deal with inactive tracks
  for id_track in track_inactive:

      if now > self.record_tracker[id_track]["expire"] or (now - self.record_tracker[id_track]["since"]) < self.minimum_valid_track_duration:
          # remove tracks that have been inactive for too long
          # recall that we still have self.record_tracker_archive
          del self.record_tracker[id_track]
          del self.box_active[id_track]
          del self.kf_active[id_track]

  # add new trackers
  for id_track in det_to_add:
      self.record_tracker[self.tracker_count] = {}
      self.record_tracker[self.tracker_count]["frame"] = {}
      self.record_tracker[self.tracker_count]["frame"][frame_id] = {}
      self.record_tracker[self.tracker_count]["frame"][frame_id]["bbox"] = bboxes[id_track]
      self.record_tracker[self.tracker_count]["id"] = self.tracker_count
      self.record_tracker[self.tracker_count]["color"] = self.list_colors[self.tracker_count % self.n_colors]
      self.record_tracker[self.tracker_count]["expire"] = now + self.duration_inactive_to_remove
      self.record_tracker[self.tracker_count]["since"] = now

      # save active box
      self.box_active[self.tracker_count] = bboxes[id_track]

      # save active Kalaman Filter
      new_kf_x = center_circle_list[id_track][0]
      new_kf_y = center_circle_list[id_track][1]
      self.kf_active[self.tracker_count] = kf_utils.init_kf_fixed_size([new_kf_x, 0.0, new_kf_y, 0.0], self.kf_time_step, self.kf_sigma_proc, self.kf_sigma_meas)

      # save archive data
      self.record_tracker_archive[self.tracker_count] = self.record_tracker[self.tracker_count]

      # set output
      prev_exist[id_track] = False
      person_id[id_track] = self.tracker_count
      person_color[id_track] = self.record_tracker[self.tracker_count]["color"]
      tracked_duration[id_track] = (now - self.record_tracker[self.tracker_count]["since"]).nanoseconds/1000000000

      self.tracker_count += 1

  return prev_exist, [int(x) for x in person_id], person_color, tracked_duration

*/
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
  detected_boxes_sub_ = this->create_subscription<track_people_msgs::msg::TrackedBoxes>("people/detected_boxes", 10, std::bind(&AbsTrackPeople::detected_boxes_cb, this, std::placeholders::_1));
  tracked_boxes_pub_ = this->create_publisher<track_people_msgs::msg::TrackedBoxes>("people/tracked_boxes", 10);
  visualization_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("people/tracking_visualization", 10);
  /*WIP
  updater = Updater(self)
*/
  double target_fps = this->declare_parameter("target_fps", 0.0);
  std::string diagnostic_name = this->declare_parameter("diagnostic_name", "PeopleTrack");
/**
  self.htd = HeaderlessTopicDiagnostic(diagnostic_name, self.updater,
                                        FrequencyStatusParam({'min': target_fps/3.0, 'max': target_fps}, 0.2, 2))
  */
}

void AbsTrackPeople::preprocess_msg(
  std::vector<std::vector<double>>* detect_results,
  std::vector<std::vector<double>>* center_bird_eye_global_list,
  const track_people_msgs::msg::TrackedBoxes& detected_boxes_msg)
{
  *detect_results = std::vector<std::vector<double>>();
  *center_bird_eye_global_list = std::vector<std::vector<double>>();
  int idx_bbox = 0;
  for (auto bbox : detected_boxes_msg.tracked_boxes) {
    detect_results->push_back(
      std::vector<double>{bbox.box.xmin, bbox.box.ymin, bbox.box.xmax, bbox.box.ymax});
    center_bird_eye_global_list->push_back(
      std::vector<double>{bbox.center3d.x, bbox.center3d.y, bbox.center3d.z});
  }
}

void AbsTrackPeople::pub_result(
  const track_people_msgs::msg::TrackedBoxes& detected_boxes_msg,
  const std::vector<int>& id_list,
  const std::vector<std::vector<double>>& color_list,
  const std::vector<double>& tracked_duration)
{
  // publish tracked boxes message
  track_people_msgs::msg::TrackedBoxes tracked_boxes_msg;
  tracked_boxes_msg.header = detected_boxes_msg.header;
  tracked_boxes_msg.camera_id = detected_boxes_msg.camera_id;
  tracked_boxes_msg.pose = detected_boxes_msg.pose;

  int idx_bbox = 0;
  for (auto bbox : detected_boxes_msg.tracked_boxes) {
    if (tracked_duration[idx_bbox] < minimum_valid_track_duration_) {
      idx_bbox++;
      continue;
    }
    track_people_msgs::msg::TrackedBox tracked_box;
    tracked_box.header = bbox.header;
    tracked_box.track_id = id_list[idx_bbox];
    tracked_box.color.r = color_list[idx_bbox][0];
    tracked_box.color.g = color_list[idx_bbox][1];
    tracked_box.color.b = color_list[idx_bbox][2];
    tracked_box.color.a = 0.0;
    tracked_box.box = bbox.box;
    tracked_box.center3d = bbox.center3d;
    tracked_boxes_msg.tracked_boxes.push_back(tracked_box);

    idx_bbox++;
  }
  tracked_boxes_pub_->publish(tracked_boxes_msg);

  RCLCPP_INFO(this->get_logger(), "camera ID = %s, number of tracked people = %d",
    detected_boxes_msg.camera_id.c_str(), tracked_boxes_msg.tracked_boxes.size());
}

void AbsTrackPeople::vis_result(
  const track_people_msgs::msg::TrackedBoxes& detected_boxes_msg,
  const std::vector<int>& id_list,
  const std::vector<std::vector<double>>& color_list,
  const std::vector<double>& tracked_duration)
{
  // publish visualization marker array for rviz
  visualization_msgs::msg::MarkerArray marker_array;
  int idx_bbox = 0;
  for (auto bbox : detected_boxes_msg.tracked_boxes) {
    if (tracked_duration[idx_bbox] < minimum_valid_track_duration_) {
      idx_bbox++;
      continue;
    }
    visualization_msgs::msg::Marker marker;
    marker.header = bbox.header;
    marker.ns = "track-people";
    marker.id = id_list[idx_bbox];
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(500000000.0 / 1000000000.0); // nanoseconds -> seconds
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.2;
    marker.pose.position = bbox.center3d;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = color_list[idx_bbox][0];
    marker.color.g = color_list[idx_bbox][1];
    marker.color.b = color_list[idx_bbox][2];
    marker.color.a = 1.0;
    marker_array.markers.push_back(marker);

    idx_bbox++;
  }
  visualization_marker_array_pub_->publish(marker_array);
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
  tracker_ = std::shared_ptr<TrackerSort3D>(new TrackerSort3D(
    iou_threshold_, iou_circle_size_, minimum_valid_track_duration_,
    duration_inactive_to_remove_));

  combined_detected_boxes_pub_ = this->create_publisher<track_people_msgs::msg::TrackedBoxes>("people/combined_detected_boxes", 10);

  buffer_ = std::map<std::string, track_people_msgs::msg::TrackedBoxes>();

/*
WIP
  plt.ion()
  plt.show()
  rclpy.spin(track_people)
*/
}

void TrackSort3dPeople::detected_boxes_cb(
  track_people_msgs::msg::TrackedBoxes detected_boxes_msg)
{
/*
WIP
  self.htd.tick()
*/
  RCLCPP_INFO(this->get_logger(), "detected_boxes_cb");

/*
WIP
  # check if tracker is initialized
  if not hasattr(self, 'tracker'):
      return
*/
  std::unique_ptr<rclcpp::Time> now = std::make_unique<rclcpp::Time>(get_clock()->now());

  // To ignore cameras which stop by accidents, remove detecion results for cameras that are not updated longer than threshold to remove track
  std::vector<std::string> delete_camera_ids;
  for (const auto& [key, value] : buffer_) {
    if ((*now - rclcpp::Time(value.header.stamp)) > rclcpp::Duration::from_seconds(tracker_->duration_inactive_to_remove_)) {
      delete_camera_ids.push_back(key);
    }
  }
  for (const auto& d : delete_camera_ids) {
    RCLCPP_INFO(this->get_logger(), "delete buffer for the camera which is not updated, camera ID = %s", d.c_str());
    buffer_.erase(d);
  }

/*
  2022.01.12: remove time check for multiple detection
  check if image is received in correct time order
  cur_detect_time_sec = detected_boxes_msg.header.stamp.to_sec()
  if cur_detect_time_sec<self.prev_detect_time_sec:
     return
*/

  buffer_[detected_boxes_msg.camera_id] = detected_boxes_msg;

  std::shared_ptr<track_people_msgs::msg::TrackedBoxes> combined_msg;

  for (const auto& [key, value] : buffer_) {
    std::shared_ptr<track_people_msgs::msg::TrackedBoxes> msg =
      std::make_shared<track_people_msgs::msg::TrackedBoxes>(buffer_[key]);
    if (combined_msg) {
      combined_msg = msg;
    } else {
      combined_msg->tracked_boxes.insert(
        combined_msg->tracked_boxes.end(),
        msg->tracked_boxes.begin(),
        msg->tracked_boxes.end());
    }
  }
  combined_msg->header.stamp = *now;

  std::vector<std::vector<double>> detect_results;
  std::vector<std::vector<double>> center_bird_eye_global_list;
  preprocess_msg(&detect_results, &center_bird_eye_global_list, *combined_msg);

  combined_detected_boxes_pub_->publish(*combined_msg);

  std::vector<bool> prev_exist;
  std::vector<int> id_list;
  std::vector<std::vector<double>> color_list;
  std::vector<double> tracked_duration;

  try {
    tracker_->track(
      &prev_exist, &id_list, &color_list, &tracked_duration,
      *now, detect_results, center_bird_eye_global_list, frame_id_);
  } catch (char* e) {
    RCLCPP_ERROR(this->get_logger(), "tracking error, %s", e);
  }

  pub_result(*combined_msg, id_list, color_list, tracked_duration);

  vis_result(*combined_msg, id_list, color_list, tracked_duration);

  frame_id_ += 1;
  // self.prev_detect_time_sec = cur_detect_time_sec
}

void TrackSort3dPeople::receiveSignal(int signal_num, int frame)
{
/*
WIP
  print("Received:", signal_num)
  sys.exit(0)


  signal.signal(signal.SIGINT, receiveSignal)
*/
}

}  // namespace track_people_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(track_people_cpp::TrackSort3dPeople)
