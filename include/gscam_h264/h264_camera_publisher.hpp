#ifndef GSCAM_H264__H264_CAMERA_PUBLISHER_HPP_
#define GSCAM_H264__H264_CAMERA_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace gscam_h264
{

class H264CameraPublisher : public rclcpp::Node
{
public:
  explicit H264CameraPublisher(const rclcpp::NodeOptions & options);
  ~H264CameraPublisher() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gscam_h264

#endif  // GSCAM_H264__H264_CAMERA_PUBLISHER_HPP_
