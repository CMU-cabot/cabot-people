
#ifndef TRACK_SORT_3D_PEOPLE_HPP_
#define TRACK_SORT_3D_PEOPLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
    std::vector<bool>* prev_exist, std::vector<int>* person_id,
    std::vector<std::vector<double>>* person_color, std::vector<double>* tracked_duration,
    const rclcpp::Time& now, const std::vector<std::vector<double>>& bboxes,
    const std::vector<std::vector<double>>& center_pos_list,
    int frame_id, int counter_penalty, bool drop_inactive_feature);
  void get_track_id(int id_track);
  void get_active_tracks_at_frame(int id_frame);

  double duration_inactive_to_remove_;
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
  double n_colors_;
  double tracker_count_;
};

class AbsTrackPeople : public rclcpp::Node
{
public:
  explicit AbsTrackPeople(
    char* node_name, rclcpp::NodeOptions options, char* device,
    int minimum_valid_track_duration);
  virtual void detected_boxes_cb(
    track_people_msgs::msg::TrackedBoxes detected_boxes_msg) = 0;
  void preprocess_msg(
    std::vector<std::vector<double>>* detect_results,
    std::vector<std::vector<double>>* center_bird_eye_global_list,
    const track_people_msgs::msg::TrackedBoxes& detected_boxes_msg);
  void pub_result(
    const track_people_msgs::msg::TrackedBoxes& detected_boxes_msg,
    const std::vector<int>& id_list,
    const std::vector<std::vector<double>>& color_list,
    const std::vector<double>& tracked_duration);
  void vis_result(
    const track_people_msgs::msg::TrackedBoxes& detected_boxes_msg,
    const std::vector<int>& id_list,
    const std::vector<std::vector<double>>& color_list,
    const std::vector<double>& tracked_duration);
private:
  int minimum_valid_track_duration_;
  char* device_;
  rclcpp::Subscription<track_people_msgs::msg::TrackedBoxes>::SharedPtr detected_boxes_sub_;
  rclcpp::Publisher<track_people_msgs::msg::TrackedBoxes>::SharedPtr tracked_boxes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_marker_array_pub_;
  int prev_detect_time_sec_;
  int updater_;
protected:
  int htd_;
  int frame_id_;
};


class TrackSort3dPeople : public AbsTrackPeople
{
public:
  explicit TrackSort3dPeople(rclcpp::NodeOptions options);
  void detected_boxes_cb(
    track_people_msgs::msg::TrackedBoxes detected_boxes_msg) override;
  void receiveSignal(int signal_num, int frame);

private:
  rclcpp::Node * nh_;

  char* device_;
  double minimum_valid_track_duration_;
  double iou_threshold_;
  double iou_circle_size_;
  double duration_inactive_to_remove_;

  std::shared_ptr<TrackerSort3D> tracker_;
  rclcpp::Publisher<track_people_msgs::msg::TrackedBoxes>::SharedPtr combined_detected_boxes_pub_;
  std::map<std::string, track_people_msgs::msg::TrackedBoxes> buffer_;
};


}  // namespace track_people_cpp


#endif  // TRACK_SORT_3D_PEOPLE_HPP_
