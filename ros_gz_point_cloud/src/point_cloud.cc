// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "point_cloud.hh"
#include <gz/common/Event.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/DepthCamera.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/RgbdCamera.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include <rclcpp/init_options.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


using namespace ros_gz_point_cloud;

/// \brief Types of sensors supported by this plugin
enum class SensorType
{
  /// \brief A camera which combines an RGB and a depth camera
  RGBD_CAMERA,

  /// \brief Depth camera
  DEPTH_CAMERA,

  /// \brief GPU lidar rays
  GPU_LIDAR
};

//////////////////////////////////////////////////
class ros_gz_point_cloud::PointCloudPrivate
{
  /// \brief Callback when the depth camera generates a new frame.
  /// This is called in the rendering thread.
  /// \param[in] _scan Depth image data
  /// \param[in] _width Image width in pixels
  /// \param[in] _height Image height in pixels
  /// \param[in] _channels Number of channels in image.
  /// \param[in] _format Image format as string.

public:
  void OnNewDepthFrame(
    const float * _scan,
    unsigned int _width, unsigned int _height,
    unsigned int _channels,
    const std::string & _format);

  /// \brief Get depth camera from rendering.
  /// \param[in] _ecm Immutable reference to ECM.

public:
  void LoadDepthCamera(const gz::sim::EntityComponentManager & _ecm);

  /// \brief Get RGB camera from rendering.
  /// \param[in] _ecm Immutable reference to ECM.

public:
  void LoadRgbCamera(const gz::sim::EntityComponentManager & _ecm);

  /// \brief Get GPU rays from rendering.
  /// \param[in] _ecm Immutable reference to ECM.

public:
  void LoadGpuRays(const gz::sim::EntityComponentManager & _ecm);

/* Intial attempt, kept for future Reference.

  // / \brief Retrieve the Scene from the Renderer.

public:
  void AcquireScene();

  // / \brief Acquire Scene Event Handle

public:
  gz::common::ConnectionPtr scene_connection_;
*/

  /// \brief Rendering scene which manages the cameras.

public:
  gz::rendering::ScenePtr scene_;

  /// \brief Entity ID for sensor within Gazebo.

public:
  gz::sim::Entity entity_;

  /// \brief Rendering depth camera

public:
  std::shared_ptr < gz::rendering::DepthCamera > depth_camera_;

  /// \brief Rendering RGB camera

public:
  std::shared_ptr < gz::rendering::Camera > rgb_camera_;

  /// \brief Rendering GPU lidar

public:
  std::shared_ptr < gz::rendering::GpuRays > gpu_rays_;

  /// \brief Keep latest image from RGB camera.

public:
  gz::rendering::Image rgb_image_;

  /// \brief Message populated with latest image from RGB camera.

public:
  sensor_msgs::msg::Image rgb_image_msg_;

  /// \brief Connection to depth frame event.

public:
  gz::common::ConnectionPtr depth_connection_;

  /// \brief Connection to GPU rays frame event.

public:
  gz::common::ConnectionPtr gpu_rays_connection_;

  /// \brief Node to publish ROS messages.

public:
  rclcpp::Node::UniquePtr rosnode_;

  /// \brief Point cloud ROS publisher.

public:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

  /// \brief Current simulation time.

public:
  std::chrono::steady_clock::duration current_time_;

  /// \brief Frame ID to put in message header. Defaults to sensor scoped name.

public:
  std::string frame_id_;

  /// \brief Render engine name

public:
  std::string engine_name_;

  /// \brief Render scene name

public:
  std::string scene_name_;

  /// \brief Type of sensor which this plugin is attached to.

public:
  SensorType type_;
};

//////////////////////////////////////////////////
PointCloud::PointCloud()
: dataPtr(std::make_unique < PointCloudPrivate > ())
{
}

//////////////////////////////////////////////////
void PointCloud::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr < const sdf::Element > & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & _eventMgr)
{
  rclcpp::Logger logger = rclcpp::get_logger("ros_gz_point_cloud");

  this->dataPtr->entity_ = _entity;

  if (_ecm.Component < gz::sim::components::RgbdCamera > (_entity) != nullptr) {
    this->dataPtr->type_ = SensorType::RGBD_CAMERA;
  } else if (_ecm.Component < gz::sim::components::DepthCamera > (_entity) != nullptr) {
    this->dataPtr->type_ = SensorType::DEPTH_CAMERA;
  } else if (_ecm.Component < gz::sim::components::GpuLidar > (_entity) != nullptr) {
    this->dataPtr->type_ = SensorType::GPU_LIDAR;
  } else {
    RCLCPP_ERROR(logger,
                 "Point cloud plugin must be attached to an RGBD camera, depth camera or GPU lidar.");
    return;
  }

  // Initialize ROS
  if (!rclcpp::ok()) {
    // TODO: Maybe the signal handlers need to be not-installed
    rclcpp::init(0, nullptr);
    RCLCPP_ERROR(logger, "Initialized ROS");
  }

  // Sensor scoped name
  auto scoped_name = gz::sim::scopedName(this->dataPtr->entity_, _ecm, "/", false);

  // ROS node
  auto ns = _sdf->Get < std::string > ("namespace", scoped_name).first;
  this->dataPtr->rosnode_ = rclcpp::Node::make_unique("gazebo", ns);

  // Publisher
  auto topic = _sdf->Get < std::string > ("topic", "points").first;
  this->dataPtr->pc_pub_ = this->dataPtr->rosnode_->create_publisher<sensor_msgs::msg::PointCloud2> 
    (topic, 1);

  // TF frame ID
  this->dataPtr->frame_id_ = _sdf->Get < std::string > ("frame_id", scoped_name).first;

  // Rendering engine and scene
  this->dataPtr->engine_name_ = _sdf->Get < std::string > ("engine", "ogre2").first;
  this->dataPtr->scene_name_ = _sdf->Get < std::string > ("scene", "scene").first;

  this->connection_ =
    _eventMgr.Connect<gz::sim::events::PostRender>(std::bind(&PointCloud::PerformRenderingOperations, this));
}

//////////////////////////////////////////////////
void PointCloud::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  this->dataPtr->current_time_ = _info.simTime;

  if (nullptr == this->dataPtr->scene_) {
    this->dataPtr->scene_ = this->scene_;

    if (nullptr == this->dataPtr->scene_)
      return;
  }

  // Get rendering objects
  if (!this->dataPtr->depth_camera_ &&
    (this->dataPtr->type_ == SensorType::RGBD_CAMERA ||
    this->dataPtr->type_ == SensorType::DEPTH_CAMERA))
  {
    this->dataPtr->LoadDepthCamera(_ecm);
  }
  if (!this->dataPtr->rgb_camera_ &&
    this->dataPtr->type_ == SensorType::RGBD_CAMERA)
  {
    this->dataPtr->LoadRgbCamera(_ecm);
  }
  if (!this->dataPtr->gpu_rays_ &&
    this->dataPtr->type_ == SensorType::GPU_LIDAR)
  {
    this->dataPtr->LoadGpuRays(_ecm);
  }
}

/* Intial attempt, kept for future Reference.
//////////////////////////////////////////////////
void PointCloudPrivate::AcquireScene() {
    // Find engine / scene
  if (nullptr == this->scene_) {
    auto engine = gz::rendering::engine(this->engine_name_);
    if (!engine) {
      return;
    }

    this->scene_ = engine->SceneByName(this->scene_name_);

    if (nullptr == this->scene_) {
      RCLCPP_INFO(this->rosnode_->get_logger(), "The scene has not been found");
      return;
    }
  }
}
*/

//////////////////////////////////////////////////
// Based on the Server Render Plugin tutorial
void PointCloud::PerformRenderingOperations()
{
  if (nullptr == this->scene_)
  {
    this->FindScene();
  }

  if (nullptr == this->scene_)
    return;
}

/////////////////////////////////////////////////
// Based on the Server Render Plugin tutorial
void PointCloud::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  // assume there is only one engine loaded
  auto engineName = this->dataPtr->engine_name_;
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }

  // Get the scene
  auto scenePtr = engine->SceneByName(this->dataPtr->scene_name_);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
      << "Using scene [" << scene_->Name() << "]" << std::endl;
  }

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }

  this->scene_ = scenePtr;
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadDepthCamera(
  const gz::sim::EntityComponentManager & _ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
    gz::sim::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name + "_depth");
  if (!sensor) {
    sensor = this->scene_->SensorByName(sensor_name);
    if (!sensor) {
      return;
    }
  }

  this->depth_camera_ =
    std::dynamic_pointer_cast < gz::rendering::DepthCamera > (sensor);
  if (!this->depth_camera_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Rendering sensor named [%s] is not a depth camera", sensor_name.c_str());
    return;
  }

  this->depth_connection_ = this->depth_camera_->ConnectNewDepthFrame(
    std::bind(
      &PointCloudPrivate::OnNewDepthFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadRgbCamera(
  const gz::sim::EntityComponentManager & _ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
    gz::sim::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name);
  if (!sensor) {
    return;
  }

  this->rgb_camera_ = std::dynamic_pointer_cast < gz::rendering::Camera > (sensor);
  if (!this->rgb_camera_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Rendering sensor named [%s] is not an RGB camera", sensor_name.c_str());
    return;
  }

  this->rgb_image_ = this->rgb_camera_->CreateImage();
}

//////////////////////////////////////////////////
void PointCloudPrivate::LoadGpuRays(
  const gz::sim::EntityComponentManager & _ecm)
{
  // Sensor name scoped from the model
  auto sensor_name =
    gz::sim::scopedName(this->entity_, _ecm, "::", false);
  sensor_name = sensor_name.substr(sensor_name.find("::") + 2);

  // Get sensor
  auto sensor = this->scene_->SensorByName(sensor_name);
  if (!sensor) {
    return;
  }

  this->gpu_rays_ =
    std::dynamic_pointer_cast < gz::rendering::GpuRays > (sensor);
  if (!this->gpu_rays_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Rendering sensor named [%s] is not a depth camera", sensor_name.c_str());
    return;
  }

  this->gpu_rays_connection_ = this->gpu_rays_->ConnectNewGpuRaysFrame(
    std::bind(
      &PointCloudPrivate::OnNewDepthFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
}

//////////////////////////////////////////////////
void PointCloudPrivate::OnNewDepthFrame(
  const float * _scan,
  unsigned int _width, unsigned int _height,
  unsigned int _channels,
  const std::string & _format)
{
  if (this->pc_pub_->get_subscription_count() <= 0 || _height == 0 || _width == 0) {
    RCLCPP_INFO(rclcpp::get_logger("ros_gz_point_cloud"), "No subscribers for the point cloud. Skipping.");
    return;
  }

  // Just sanity check, but don't prevent publishing
  if (this->type_ == SensorType::RGBD_CAMERA && _channels != 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Expected depth image to have 1 channel, but it has [%i]", _channels);
  }
  if (this->type_ == SensorType::GPU_LIDAR && _channels != 3) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Expected GPU rays to have 3 channels, but it has [%i]", _channels);
  }
  if ((this->type_ == SensorType::RGBD_CAMERA ||
    this->type_ == SensorType::DEPTH_CAMERA) && _format != "FLOAT32")
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Expected depth image to have [FLOAT32] format, but it has [%s]", _format.c_str());
  }
  if (this->type_ == SensorType::GPU_LIDAR && _format != "PF_FLOAT32_RGB") {
    RCLCPP_WARN(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Expected GPU rays to have [PF_FLOAT32_RGB] format, but it has [%s]", _format.c_str());
  }

  RCLCPP_INFO(
      rclcpp::get_logger("ros_gz_point_cloud"),
      "Processed as expected preparing Point Cloud"
  );

  // Fill message
  // Logic borrowed from
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_depth_camera.cpp
  auto sec_nsec = gz::math::durationToSecNsec(this->current_time_);

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = this->frame_id_;
  msg.header.stamp.sec = sec_nsec.first;
  msg.header.stamp.nanosec = sec_nsec.second;
  msg.width = _width;
  msg.height = _height;
  msg.row_step = msg.point_step * _width;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(_width * _height);

  sensor_msgs::PointCloud2Iterator < float > iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator < float > iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator < float > iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator < uint8_t > iter_r(msg, "r");
  sensor_msgs::PointCloud2Iterator < uint8_t > iter_g(msg, "g");
  sensor_msgs::PointCloud2Iterator < uint8_t > iter_b(msg, "b");

  if (this->rgb_camera_) {
    this->rgb_camera_->Capture(this->rgb_image_);
    sensor_msgs::fillImage(
      this->rgb_image_msg_, sensor_msgs::image_encodings::RGB8, _height,
      _width, 3 * _width, this->rgb_image_.Data < unsigned char > ());
  }

  // For depth calculation from image
  double fl {0.0};
  if (nullptr != this->depth_camera_) {
    auto hfov = this->depth_camera_->HFOV().Radian();
    fl = _width / (2.0 * tan(hfov / 2.0));
  }

  // For depth calculation from laser scan
  double angle_step {0.0};
  double vertical_angle_step {0.0};
  double inclination {0.0};
  double azimuth {0.0};
  if (nullptr != this->gpu_rays_) {
    angle_step = (this->gpu_rays_->AngleMax() - this->gpu_rays_->AngleMin()).Radian() /
      (this->gpu_rays_->RangeCount() - 1);
    vertical_angle_step = (this->gpu_rays_->VerticalAngleMax() -
      this->gpu_rays_->VerticalAngleMin()).Radian() / (this->gpu_rays_->VerticalRangeCount() - 1);

    // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
    inclination = this->gpu_rays_->VerticalAngleMin().Radian();
    azimuth = this->gpu_rays_->AngleMin().Radian();
  }

  // For color calculation
  uint8_t * image_src;
  if (nullptr != this->rgb_camera_) {
    image_src = (uint8_t *)(&(this->rgb_image_msg_.data[0]));
  }

  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < _height; ++j) {
    double p_angle {0.0};
    if (fl > 0 && _height > 1) {
      p_angle = atan2((double)j - 0.5 * (double)(_height - 1), fl);
    }

    if (nullptr != this->gpu_rays_) {
      azimuth = this->gpu_rays_->AngleMin().Radian();
    }
    for (uint32_t i = 0; i < _width;
      ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
      // Index of current point
      auto index = j * _width * _channels + i * _channels;
      double depth = _scan[index];

      double y_angle {0.0};
      if (fl > 0 && _width > 1) {
        y_angle = atan2((double)i - 0.5 * (double)(_width - 1), fl);
      }

      if (nullptr != this->depth_camera_) {
        // in optical frame
        // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
        // to urdf, where the *_optical_frame should have above relative
        // rotation from the physical camera *_frame
        *iter_x = depth * tan(y_angle);
        *iter_y = depth * tan(p_angle);
        *iter_z = depth;

        // Clamp according to REP 117
        if (depth > this->depth_camera_->FarClipPlane()) {
          *iter_z = gz::math::INF_D;
          msg.is_dense = false;
        }
        if (depth < this->depth_camera_->NearClipPlane()) {
          *iter_z = -gz::math::INF_D;
          msg.is_dense = false;
        }
      } else if (nullptr != this->gpu_rays_) {
        // Convert spherical coordinates to Cartesian for pointcloud
        // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
        *iter_x = depth * cos(inclination) * cos(azimuth);
        *iter_y = depth * cos(inclination) * sin(azimuth);
        *iter_z = depth * sin(inclination);
      }

      // Put image color data for each point
      if (this->rgb_image_msg_.data.size() == _height * _width * 3) {
        // color
        *iter_r = image_src[i * 3 + j * _width * 3 + 0];
        *iter_g = image_src[i * 3 + j * _width * 3 + 1];
        *iter_b = image_src[i * 3 + j * _width * 3 + 2];
      } else if (this->rgb_image_msg_.data.size() == _height * _width) {
        // mono?
        *iter_r = image_src[i + j * _width];
        *iter_g = image_src[i + j * _width];
        *iter_b = image_src[i + j * _width];
      } else {
        // no image
        *iter_r = 0;
        *iter_g = 0;
        *iter_b = 0;
      }
      azimuth += angle_step;
    }
    inclination += vertical_angle_step;
  }

  this->pc_pub_->publish(msg);
}

GZ_ADD_PLUGIN(
  ros_gz_point_cloud::PointCloud,
  gz::sim::System,
  ros_gz_point_cloud::PointCloud::ISystemConfigure,
  ros_gz_point_cloud::PointCloud::ISystemPostUpdate)
