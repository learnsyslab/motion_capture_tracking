#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
// Motion Capture
#include <libmotioncapture/motioncapture.h>

// Reference frame that both the TF tree and the stamped poses are expressed in.
static constexpr const char *kWorldFrameId = "world";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_capture_tracking_node");
  node->declare_parameter<std::string>("type", "vicon");
  node->declare_parameter<std::string>("hostname", "localhost");
  // ToDo: maybe we should log all tracked bodies in file.
  node->declare_parameter<std::string>("logfilepath", "");
  node->declare_parameter<bool>("publish_stamped_poses", false);
  node->declare_parameter<bool>("broadcast_tf", false);
  node->declare_parameter<bool>("use_host_time", true);
  node->declare_parameter<double>("update_rate", 0.0);

  std::vector<std::string> publish_poses_for; // list from ros param that contains topic assignment
  std::vector<std::string> publish_poses_for_clean; // clean list that only contains the target names
  std::unordered_map<std::string, std::string> map_target_topic_name;
  node->declare_parameter<std::vector<std::string> >("publish_stamped_poses_for", publish_poses_for);

  const std::string motionCaptureType = node->get_parameter("type").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "motion capture type: " << motionCaptureType);
  const std::string motionCaptureHostname = node->get_parameter("hostname").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "hostname: " << motionCaptureHostname);
  auto use_host_time = node->get_parameter("use_host_time").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(), "Use host time: " << (use_host_time ? "true" : "false"));
  const auto publish_stamped_poses = node->get_parameter("publish_stamped_poses").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(), "Publish stamped poses: " << (publish_stamped_poses ? "true" : "false"));
  const auto broadcast_tf = node->get_parameter("broadcast_tf").as_bool();
  RCLCPP_INFO_STREAM(node->get_logger(), "Broadcast TF: " << (broadcast_tf ? "true" : "false"));
  const double update_rate = node->get_parameter("update_rate").as_double();
  RCLCPP_INFO_STREAM(node->get_logger(), "Update rate (Hz): " << update_rate);

  publish_poses_for = node->get_parameter("publish_stamped_poses_for").as_string_array();
  auto publish_stamped_poses_all = false;
  if (publish_stamped_poses && publish_poses_for.empty()) {
    RCLCPP_WARN(node->get_logger(),
                "Publish stamped poses set but no target has been assigned, publishing all targets' poses");
    publish_stamped_poses_all = true;
  }
  if (!publish_poses_for.empty()) {
    publish_poses_for_clean.reserve(publish_poses_for.size());
    RCLCPP_INFO(node->get_logger(), "Publish stamped poses for the following targets:");
    for (const auto &target: publish_poses_for) {
      const size_t pos = target.find(':');
      if (pos != std::string::npos) {
        const auto target_name = target.substr(0, pos);
        const auto target_topic = target.substr(pos + 1);
        map_target_topic_name.emplace(
          target_name,
          target_topic);
        RCLCPP_INFO_STREAM(node->get_logger(), "--- " << target_name << " with topic: " << target_topic);
        publish_poses_for_clean.push_back(target_name);
      } else {
        publish_poses_for_clean.push_back(target);
        RCLCPP_INFO_STREAM(node->get_logger(), "--- " << target);
      }
    }
  }

  std::string logFilePath = node->get_parameter("logfilepath").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "logfilepath: " << logFilePath);

  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
      map_target_pose_publishers;
  std::unordered_set<std::string> publish_poses_for_set(publish_poses_for_clean.begin(), publish_poses_for_clean.end());

  RCLCPP_INFO(node->get_logger(), " ****** Starting MotionCapture Interface ******* ");

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = motionCaptureHostname;

  std::unique_ptr<libmotioncapture::MotionCapture> mocap(
    libmotioncapture::MotionCapture::connect(motionCaptureType, cfg));

  // Not every backend reports the camera timestamp; the base implementation returns 0, which
  // would stamp every message at the epoch.
  if (!use_host_time && !mocap->supportsTimeStamp()) {
    RCLCPP_WARN_STREAM(node->get_logger(),
                       "use_host_time is false, but the '" << motionCaptureType
                       << "' backend does not report frame timestamps. Falling back to host time.");
    use_host_time = true;
  }

  // prepare TF broadcaster
  tf2_ros::TransformBroadcaster tfbroadcaster(node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // lambda functions
  auto ensure_map_target_publisher = [&map_target_pose_publishers, &map_target_topic_name, node
      ](const std::string &name) {
    if (!map_target_pose_publishers.contains(name)) {
      std::string topic;
      if (map_target_topic_name.contains(name))
        topic = map_target_topic_name.at(name);
      else
        topic = "stamped_pose_" + name;
      auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic,
                                                                         rclcpp::SystemDefaultsQoS());
      map_target_pose_publishers.emplace(
        name,
        std::move(pub));
    }
  };

  auto generate_and_publish_stamped_pose = [&map_target_pose_publishers](const std::string &name,
                                                                         const rclcpp::Time &time,
                                                                         const auto &rigidBody) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = time;
    // header.frame_id is the frame the pose is expressed in, not the name of the body itself.
    pose.header.frame_id = kWorldFrameId;

    pose.pose.position.x = rigidBody.position().x();
    pose.pose.position.y = rigidBody.position().y();
    pose.pose.position.z = rigidBody.position().z();

    pose.pose.orientation.x = rigidBody.rotation().x();
    pose.pose.orientation.y = rigidBody.rotation().y();
    pose.pose.orientation.z = rigidBody.rotation().z();
    pose.pose.orientation.w = rigidBody.rotation().w();
    map_target_pose_publishers.at(name)->publish(pose);
  };

  // Frames are always consumed as fast as the motion capture system delivers them. The Vicon
  // backend streams in ServerPush mode, so a consumer that is slower than the stream rate does
  // not drop frames, it falls behind -- and the published poses get progressively staler.
  // "update_rate" therefore only controls how often a received frame is published, never how
  // often frames are read.
  const bool publish_every_frame = update_rate <= 0.0;
  const auto publish_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(publish_every_frame ? 0.0 : 1.0 / update_rate));

  if (publish_every_frame) {
    RCLCPP_INFO(node->get_logger(), "update_rate is 0: publishing on every received frame.");
  }

  // libmotioncapture does not expose the frame rate, so estimate it from the first frames. This
  // is only used to warn about an update_rate that cannot be hit exactly; the pacing below does
  // not depend on the estimate.
  constexpr size_t kRateEstimationFrames = 200;
  size_t frames_seen = 0;
  bool startup_checks_done = false;
  std::chrono::steady_clock::time_point rate_window_start;
  std::unordered_set<std::string> seen_target_names;

  auto next_publish = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    // Get a frame
    mocap->waitForNextFrame();
    const auto frame_arrival = std::chrono::steady_clock::now();

    // Service ROS callbacks on every frame, not just on the ones that get published.
    rclcpp::spin_some(node);

    if (!startup_checks_done) {
      if (frames_seen == 0) {
        rate_window_start = frame_arrival;
      } else if (frames_seen >= kRateEstimationFrames) {
        const double elapsed = std::chrono::duration<double>(frame_arrival - rate_window_start).count();
        const double mocap_rate = elapsed > 0.0 ? frames_seen / elapsed : 0.0;
        RCLCPP_INFO_STREAM(node->get_logger(), "Measured motion capture frame rate: " << mocap_rate << " Hz");

        if (!publish_every_frame && mocap_rate > 0.0) {
          if (update_rate > mocap_rate) {
            RCLCPP_WARN_STREAM(node->get_logger(),
                               "update_rate (" << update_rate << " Hz) is higher than the motion capture "
                               "frame rate (" << mocap_rate << " Hz); publishing on every frame instead.");
          } else {
            const double ratio = mocap_rate / update_rate;
            if (std::abs(ratio - std::round(ratio)) > 0.05) {
              RCLCPP_WARN_STREAM(node->get_logger(),
                                 "update_rate (" << update_rate << " Hz) is not an integer divisor of the "
                                 "motion capture frame rate (" << mocap_rate << " Hz); the interval between "
                                 "published frames will vary by up to one frame.");
            }
          }
        }

        // A target name that never shows up is almost always a typo in the config, and is
        // otherwise silently ignored.
        for (const auto &target: publish_poses_for_set) {
          if (!seen_target_names.contains(target)) {
            RCLCPP_WARN_STREAM(node->get_logger(),
                               "Configured target '" << target << "' has not been reported by the motion "
                               "capture system; no pose will be published for it.");
          }
        }
        seen_target_names.clear();
        startup_checks_done = true;
      }
      ++frames_seen;
    }

    // Decide whether this frame is published. Deadlines are accumulated rather than reset to
    // the current time, so an exact divisor of the frame rate stays exact.
    if (!publish_every_frame) {
      if (frame_arrival < next_publish) {
        continue;
      }
      next_publish += publish_period;
      if (next_publish <= frame_arrival) {
        // Fell behind (stall, or update_rate above the frame rate) -- resynchronise instead of
        // publishing a burst to catch up.
        next_publish = frame_arrival + publish_period;
      }
    }

    rclcpp::Time time;
    if (use_host_time)
      time = node->now();
    else
      // libmotioncapture reports microseconds, rclcpp::Time takes nanoseconds.
      time = rclcpp::Time(static_cast<int64_t>(mocap->timeStamp()) * 1000);
    if (broadcast_tf) {
      transforms.clear();
      transforms.reserve(mocap->rigidBodies().size());
    }
    for (const auto &iter: mocap->rigidBodies()) {
      const auto &rigidBody = iter.second;
      const auto name = rigidBody.name();

      if (!startup_checks_done) {
        seen_target_names.insert(name);
      }

      if (publish_stamped_poses &&
          (publish_stamped_poses_all || publish_poses_for_set.contains(name))) {
        ensure_map_target_publisher(name);
        generate_and_publish_stamped_pose(name, time, rigidBody);
      }

      if (broadcast_tf) {
        transforms.resize(transforms.size() + 1);
        transforms.back().header.stamp = time;
        transforms.back().header.frame_id = kWorldFrameId;
        transforms.back().child_frame_id = name;
        transforms.back().transform.translation.x = rigidBody.position().x();
        transforms.back().transform.translation.y = rigidBody.position().y();
        transforms.back().transform.translation.z = rigidBody.position().z();
        transforms.back().transform.rotation.x = rigidBody.rotation().x();
        transforms.back().transform.rotation.y = rigidBody.rotation().y();
        transforms.back().transform.rotation.z = rigidBody.rotation().z();
        transforms.back().transform.rotation.w = rigidBody.rotation().w();
      }
    }

    if (broadcast_tf && !transforms.empty()) {
      // Since RViz and others can't handle nan's, report a fake orientation if needed
      for (auto &tf: transforms) {
        if (std::isnan(tf.transform.rotation.x)) {
          tf.transform.rotation.x = 0;
          tf.transform.rotation.y = 0;
          tf.transform.rotation.z = 0;
          tf.transform.rotation.w = 1;
        }
      }
      // send TF, once per frame rather than once per rigid body
      tfbroadcaster.sendTransform(transforms);
    }
  }

  rclcpp::shutdown();
  return 0;
}
