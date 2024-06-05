
#ifndef TRACK_SORT_3D_PEOPLE_HPP_
#define TRACK_SORT_3D_PEOPLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <track_people_msgs/msg/tracked_boxes.hpp>

namespace track_people_cpp
{
class TrackerSort3D {
public:
  TrackerSort3D(
    double iou_threshold, double iou_circle_size,
    double minimum_valid_track_duration, double duration_inactive_to_remove,
    int n_colors);

  void track(
    int* prev_exist, int* person_id, int* person_color, int* trackeduration, 
    int now, int bboxes, int center_pos_list, int frame_id,
    int counter_penalty, bool drop_inactive_feature);
  void get_track_id(int id_track);
  void get_active_tracks_at_frame(int id_frame);
private:
  double kf_time_step_;
  double kf_sigma_proc_;
  double kf_sigma_meas_;
  double sigma_l_;
  double sigma_h_;

  std::vector<int> record_tracker_;
  std::vector<int> box_active_;
  std::vector<int> kf_active_;
  std::vector<int> record_tracker_archive_;

  double iou_threshold_;
  double iou_circle_size_;
  double minimum_valid_track_duration_;
  double duration_inactive_to_remove_;
  double n_colors_;
  double tracker_count_;
};

class AbsTrackPeople : public rclcpp::Node
{
public:
  explicit AbsTrackPeople(
    char* node_name, rclcpp::NodeOptions options, char* device,
    int minimum_valid_track_duration);
  void detected_boxes_cb(int detected_boxes_msg);
  void preprocess_msg(int detected_boxes_msg);
  void pub_result(
    int detected_boxes_msg, int id_list, int color_list, int tracked_duration);
  void vis_result(
    int detected_boxes_msg, int id_list, int color_list, int tracked_duration);
private:
  int minimum_valid_track_duration_;
  char* device_;
  int detected_boxes_sub_;
  int tracked_boxes_pub_;
  int visualization_marker_array_pub_;
  int frame_id_;
  int prev_detect_time_sec_;
  int updater_;
  int htd_;
};


class TrackSort3dPeople : public AbsTrackPeople
{
public:
  explicit TrackSort3dPeople(rclcpp::NodeOptions options);
  void detected_boxes_cb(int detected_boxes_msg);
  void receiveSignal(int signal_num, int frame);

private:
  rclcpp::Node * nh_;

  char* device_;
  double minimum_valid_track_duration_;
  double iou_threshold_;
  double iou_circle_size_;
  double duration_inactive_to_remove_;

  std::shared_ptr<TrackerSort3D> tracker;
  rclcpp::Publisher<track_people_msgs::msg::TrackedBoxes>::SharedPtr combined_detected_boxes_pub_;
};


}  // namespace track_people_cpp


#endif  // TRACK_SORT_3D_PEOPLE_HPP_
