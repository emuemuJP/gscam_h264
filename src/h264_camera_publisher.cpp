#include "gscam_h264/h264_camera_publisher.hpp"

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "sensor_msgs/msg/compressed_image.hpp"

namespace gscam_h264
{

struct CameraPipeline
{
  int cam_id;
  std::string device;
  GstElement * pipeline = nullptr;
  GstElement * sink = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
  std::thread thread;
  std::atomic<bool> stop{false};
  int64_t time_offset = 0;  // signed to avoid unsigned underflow
  std::string frame_id;
};

static std::vector<std::string> split(const std::string & s, char delim)
{
  std::vector<std::string> tokens;
  std::istringstream stream(s);
  std::string token;
  while (std::getline(stream, token, delim)) {
    size_t start = token.find_first_not_of(" \t");
    size_t end = token.find_last_not_of(" \t");
    if (start != std::string::npos) {
      tokens.push_back(token.substr(start, end - start + 1));
    }
  }
  return tokens;
}

class H264CameraPublisher::Impl
{
public:
  explicit Impl(rclcpp::Node * node);
  ~Impl();

private:
  rclcpp::Node * node_;

  int camera_count_;
  std::string device_;
  std::string devices_;
  std::string device_base_;
  int device_index_;
  int width_;
  int height_;
  int fps_;
  int bitrate_;
  int iframeinterval_;
  int io_mode_;
  bool use_pts_stamp_;
  std::string topic_prefix_;

  std::vector<std::unique_ptr<CameraPipeline>> cameras_;

  std::string build_pipeline_string(const std::string & device) const;

  // Phase 1: parse pipeline + transition to PAUSED only
  bool create_and_pause(CameraPipeline & cam);

  // Phase 3: transition a single camera PAUSED -> PLAYING
  bool set_playing(CameraPipeline & cam);

  void start_thread(CameraPipeline & cam);
  void stop_camera(CameraPipeline & cam);
  void log_bus_messages(CameraPipeline & cam);
  void cleanup_failed_pipeline(CameraPipeline & cam);
  void frame_loop(CameraPipeline & cam);
};

H264CameraPublisher::Impl::Impl(rclcpp::Node * node)
: node_(node)
{
  camera_count_ = node_->declare_parameter("camera_count", 6);
  device_ = node_->declare_parameter("device", std::string(""));
  devices_ = node_->declare_parameter("devices", std::string(""));
  device_base_ = node_->declare_parameter("device_base", std::string("/dev/video"));
  device_index_ = node_->declare_parameter("device_index", 0);
  width_ = node_->declare_parameter("width", 1920);
  height_ = node_->declare_parameter("height", 1280);
  fps_ = node_->declare_parameter("fps", 30);
  bitrate_ = node_->declare_parameter("bitrate", 8000000);
  iframeinterval_ = node_->declare_parameter("iframeinterval", 30);
  io_mode_ = node_->declare_parameter("io_mode", 2);
  use_pts_stamp_ = node_->declare_parameter("use_pts_stamp", true);
  topic_prefix_ = node_->declare_parameter("topic_prefix", std::string("/cam"));

  RCLCPP_INFO(node_->get_logger(),
    "H264CameraPublisher: camera_count=%d, device='%s', devices='%s', "
    "device_base=%s, device_index=%d, %dx%d@%dfps, bitrate=%d, "
    "iframeinterval=%d, io_mode=%d, use_pts_stamp=%s",
    camera_count_, device_.c_str(), devices_.c_str(),
    device_base_.c_str(), device_index_, width_, height_, fps_,
    bitrate_, iframeinterval_, io_mode_,
    use_pts_stamp_ ? "true" : "false");

  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
    RCLCPP_INFO(node_->get_logger(), "GStreamer initialized: %s", gst_version_string());
  }

  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.durability_volatile();

  // Build cam specs
  struct CamSpec {
    int id;
    std::string device;
  };
  std::vector<CamSpec> cam_specs;

  if (!devices_.empty()) {
    auto tokens = split(devices_, ',');
    for (size_t i = 0; i < tokens.size(); ++i) {
      auto & tok = tokens[i];
      auto colon = tok.find(':');
      if (colon != std::string::npos) {
        int id = std::stoi(tok.substr(0, colon));
        std::string path = tok.substr(colon + 1);
        cam_specs.push_back({id, path});
      } else {
        cam_specs.push_back({static_cast<int>(i), tok});
      }
    }
  } else if (!device_.empty() && camera_count_ == 1) {
    cam_specs.push_back({device_index_, device_});
  } else {
    for (int i = 0; i < camera_count_; ++i) {
      int id = device_index_ + i;
      cam_specs.push_back({id, device_base_ + std::to_string(id)});
    }
  }

  cameras_.reserve(cam_specs.size());

  // ---- Phase 1: create all pipelines and reach PAUSED ----
  //   No stream is flowing yet — only driver/HW init.
  for (size_t i = 0; i < cam_specs.size(); ++i) {
    auto & spec = cam_specs[i];
    auto cam = std::make_unique<CameraPipeline>();
    cam->cam_id = spec.id;
    cam->device = spec.device;

    std::string topic = topic_prefix_ + std::to_string(cam->cam_id) + "/h264";
    cam->publisher = node_->create_publisher<sensor_msgs::msg::CompressedImage>(topic, qos);
    cam->frame_id = "cam" + std::to_string(cam->cam_id) + "_frame";

    RCLCPP_INFO(node_->get_logger(), "[Phase1] camera %d: device=%s, topic=%s",
      cam->cam_id, cam->device.c_str(), topic.c_str());

    if (create_and_pause(*cam)) {
      RCLCPP_INFO(node_->get_logger(), "[Phase1] camera %d: PAUSED ok", cam->cam_id);
      cameras_.push_back(std::move(cam));
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "[Phase1] camera %d (%s): failed, skipping",
        cam->cam_id, cam->device.c_str());
    }

    // Serialise Tegra driver init
    if (i + 1 < cam_specs.size()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  RCLCPP_INFO(node_->get_logger(), "[Phase1] %zu / %zu pipelines paused",
    cameras_.size(), cam_specs.size());

  // ---- Phase 2: start all frame_loop threads ----
  //   pull_sample will block until PLAYING, so this is safe.
  for (auto & cam : cameras_) {
    start_thread(*cam);
  }
  RCLCPP_INFO(node_->get_logger(), "[Phase2] %zu threads started (blocking on pull_sample)",
    cameras_.size());

  // ---- Phase 3: transition to PLAYING one by one ----
  //   No camera is streaming while another is being initialised.
  size_t playing_count = 0;
  for (size_t i = 0; i < cameras_.size(); ++i) {
    auto & cam = cameras_[i];
    if (set_playing(*cam)) {
      ++playing_count;
      RCLCPP_INFO(node_->get_logger(), "[Phase3] camera %d: PLAYING", cam->cam_id);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[Phase3] camera %d: PLAYING failed", cam->cam_id);
    }
    if (i + 1 < cameras_.size()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Startup complete: %zu / %zu cameras streaming",
    playing_count, cam_specs.size());
}

H264CameraPublisher::Impl::~Impl()
{
  for (auto & cam : cameras_) {
    stop_camera(*cam);
  }
  cameras_.clear();
}

std::string H264CameraPublisher::Impl::build_pipeline_string(
  const std::string & device) const
{
  std::ostringstream ss;
  ss << "v4l2src device=" << device
     << " io-mode=" << io_mode_
     << " do-timestamp=true"
     << " use-libv4l2=false"
     << " ! video/x-raw,format=UYVY"
     << ",width=" << width_
     << ",height=" << height_
     << ",framerate=" << fps_ << "/1"
     << " ! nvvidconv"
     << " ! video/x-raw(memory:NVMM),format=NV12"
     << ",width=" << width_
     << ",height=" << height_
     << ",framerate=" << fps_ << "/1"
     << " ! nvv4l2h264enc"
     << " bitrate=" << bitrate_
     << " iframeinterval=" << iframeinterval_
     << " insert-sps-pps=true"
     << " ! h264parse config-interval=1"
     << " ! video/x-h264,stream-format=byte-stream,alignment=au"
     << " ! queue max-size-buffers=1 leaky=downstream"
     << " max-size-time=0 max-size-bytes=0"
     << " ! appsink name=sink max-buffers=1 drop=true sync=false";
  return ss.str();
}

void H264CameraPublisher::Impl::log_bus_messages(CameraPipeline & cam)
{
  GstBus * bus = gst_element_get_bus(cam.pipeline);
  if (!bus) {
    return;
  }
  while (true) {
    GstMessage * msg = gst_bus_pop_filtered(bus,
      static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING));
    if (!msg) {
      break;
    }
    GError * err = nullptr;
    gchar * debug_info = nullptr;
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
      gst_message_parse_error(msg, &err, &debug_info);
      RCLCPP_ERROR(node_->get_logger(), "Camera %d GStreamer ERROR: %s",
        cam.cam_id, err->message);
      if (debug_info) {
        RCLCPP_ERROR(node_->get_logger(), "Camera %d debug: %s",
          cam.cam_id, debug_info);
      }
    } else {
      gst_message_parse_warning(msg, &err, &debug_info);
      RCLCPP_WARN(node_->get_logger(), "Camera %d GStreamer WARNING: %s",
        cam.cam_id, err->message);
    }
    if (err) {
      g_error_free(err);
    }
    g_free(debug_info);
    gst_message_unref(msg);
  }
  gst_object_unref(bus);
}

void H264CameraPublisher::Impl::cleanup_failed_pipeline(CameraPipeline & cam)
{
  log_bus_messages(cam);
  gst_element_set_state(cam.pipeline, GST_STATE_NULL);
  if (cam.sink) {
    gst_object_unref(cam.sink);
    cam.sink = nullptr;
  }
  gst_object_unref(cam.pipeline);
  cam.pipeline = nullptr;
}

bool H264CameraPublisher::Impl::create_and_pause(CameraPipeline & cam)
{
  if (access(cam.device.c_str(), R_OK) != 0) {
    RCLCPP_ERROR(node_->get_logger(),
      "Camera %d: device %s is not accessible",
      cam.cam_id, cam.device.c_str());
    return false;
  }

  std::string pipeline_str = build_pipeline_string(cam.device);
  RCLCPP_INFO(node_->get_logger(), "Camera %d pipeline: %s",
    cam.cam_id, pipeline_str.c_str());

  GError * error = nullptr;
  cam.pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
  if (!cam.pipeline) {
    RCLCPP_ERROR(node_->get_logger(), "Camera %d: gst_parse_launch failed: %s",
      cam.cam_id, error ? error->message : "unknown error");
    if (error) {
      g_error_free(error);
    }
    return false;
  }
  if (error) {
    RCLCPP_WARN(node_->get_logger(), "Camera %d: pipeline parse warning: %s",
      cam.cam_id, error->message);
    g_error_free(error);
  }

  cam.sink = gst_bin_get_by_name(GST_BIN(cam.pipeline), "sink");
  if (!cam.sink) {
    RCLCPP_ERROR(node_->get_logger(), "Camera %d: appsink 'sink' not found",
      cam.cam_id);
    gst_object_unref(cam.pipeline);
    cam.pipeline = nullptr;
    return false;
  }

  // Calibrate GStreamer clock -> ROS time (signed to avoid underflow)
  GstClock * clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  cam.time_offset = node_->now().nanoseconds() - static_cast<int64_t>(ct);

  // NULL -> PAUSED only (no stream flows yet)
  RCLCPP_INFO(node_->get_logger(), "Camera %d: -> PAUSED ...", cam.cam_id);
  GstStateChangeReturn ret = gst_element_set_state(cam.pipeline, GST_STATE_PAUSED);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(node_->get_logger(),
      "Camera %d: set_state PAUSED returned FAILURE", cam.cam_id);
    cleanup_failed_pipeline(cam);
    return false;
  }
  ret = gst_element_get_state(cam.pipeline, nullptr, nullptr, 10 * GST_SECOND);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(node_->get_logger(),
      "Camera %d: failed to reach PAUSED", cam.cam_id);
    cleanup_failed_pipeline(cam);
    return false;
  }

  return true;
}

bool H264CameraPublisher::Impl::set_playing(CameraPipeline & cam)
{
  RCLCPP_INFO(node_->get_logger(), "Camera %d: -> PLAYING ...", cam.cam_id);
  GstStateChangeReturn ret = gst_element_set_state(cam.pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(node_->get_logger(),
      "Camera %d: set_state PLAYING returned FAILURE", cam.cam_id);
    log_bus_messages(cam);
    return false;
  }
  ret = gst_element_get_state(cam.pipeline, nullptr, nullptr, 10 * GST_SECOND);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(node_->get_logger(),
      "Camera %d: failed to reach PLAYING", cam.cam_id);
    log_bus_messages(cam);
    return false;
  }
  return true;
}

void H264CameraPublisher::Impl::start_thread(CameraPipeline & cam)
{
  cam.stop = false;
  cam.thread = std::thread([this, &cam]() { frame_loop(cam); });
}

void H264CameraPublisher::Impl::stop_camera(CameraPipeline & cam)
{
  cam.stop = true;

  // Send EOS so pull_sample unblocks
  if (cam.pipeline) {
    gst_element_send_event(cam.pipeline, gst_event_new_eos());
  }

  if (cam.thread.joinable()) {
    cam.thread.join();
  }

  if (cam.sink) {
    gst_object_unref(cam.sink);
    cam.sink = nullptr;
  }

  if (cam.pipeline) {
    gst_element_set_state(cam.pipeline, GST_STATE_NULL);
    gst_object_unref(cam.pipeline);
    cam.pipeline = nullptr;
  }

  RCLCPP_INFO(node_->get_logger(), "Camera %d stopped", cam.cam_id);
}

void H264CameraPublisher::Impl::frame_loop(CameraPipeline & cam)
{
  RCLCPP_INFO(node_->get_logger(), "Camera %d: frame_loop waiting for PLAYING", cam.cam_id);

  while (!cam.stop && rclcpp::ok()) {
    GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(cam.sink));
    if (!sample) {
      if (gst_app_sink_is_eos(GST_APP_SINK(cam.sink))) {
        RCLCPP_WARN(node_->get_logger(), "Camera %d: EOS received", cam.cam_id);
        break;
      }
      RCLCPP_WARN(node_->get_logger(), "Camera %d: null sample, retrying", cam.cam_id);
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(100ms);
      continue;
    }

    GstBuffer * buf = gst_sample_get_buffer(sample);
    if (!buf) {
      gst_sample_unref(sample);
      continue;
    }

    GstMapInfo info;
    if (!gst_buffer_map(buf, &info, GST_MAP_READ)) {
      RCLCPP_WARN(node_->get_logger(), "Camera %d: failed to map buffer", cam.cam_id);
      gst_sample_unref(sample);
      continue;
    }

    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();

    if (use_pts_stamp_ && GST_BUFFER_PTS_IS_VALID(buf)) {
      GstClockTime bt = gst_element_get_base_time(cam.pipeline);
      int64_t stamp_ns = static_cast<int64_t>(buf->pts + bt) + cam.time_offset;
      if (stamp_ns < 0) {
        stamp_ns = 0;
      }
      msg->header.stamp = rclcpp::Time(stamp_ns);
    } else {
      msg->header.stamp = node_->now();
    }

    msg->header.frame_id = cam.frame_id;
    msg->format = "h264";

    msg->data.resize(info.size);
    std::memcpy(msg->data.data(), info.data, info.size);

    cam.publisher->publish(std::move(msg));

    gst_buffer_unmap(buf, &info);
    gst_sample_unref(sample);
  }

  RCLCPP_INFO(node_->get_logger(), "Camera %d: frame_loop stopped", cam.cam_id);
}

H264CameraPublisher::H264CameraPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("h264_camera_publisher", options),
  impl_(std::make_unique<Impl>(this))
{
}

H264CameraPublisher::~H264CameraPublisher()
{
  impl_.reset();
}

}  // namespace gscam_h264
