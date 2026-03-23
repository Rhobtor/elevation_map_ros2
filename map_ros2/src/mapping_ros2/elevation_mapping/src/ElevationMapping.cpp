#define BOOST_BIND_NO_PLACEHOLDERS

#include <cmath>
#include <string>
#include <array>
#include <algorithm>
#include <sstream>
#include <exception>
#include <chrono>
#include <limits>

#include <grid_map_msgs/msg/grid_map.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

#include "elevation_mapping/layer_tools.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace elevation_mapping {

ElevationMapping::ElevationMapping(std::shared_ptr<rclcpp::Node>& nodeHandle) :
      nodeHandle_(nodeHandle),
      inputSources_(nodeHandle_),
      robotPoseCacheSize_(200),
      // transformListener_(transformBuffer_),
      map_(nodeHandle),
      robotMotionMapUpdater_(nodeHandle),
      ignoreRobotMotionUpdates_(false),
      updatesEnabled_(true),
      maxNoUpdateDuration_(rclcpp::Duration::from_seconds(0.0)),
      timeTolerance_(rclcpp::Duration::from_seconds(0.0)),
      fusedMapPublishTimerDuration_(rclcpp::Duration::from_seconds(0.0)),
      isContinuouslyFusing_(false),
      visibilityCleanupTimerDuration_(rclcpp::Duration::from_seconds(0.0)),
      receivedFirstMatchingPointcloudAndPose_(false),
      initializeElevationMap_(true),
      initializationMethod_(0),
      lengthInXInitSubmap_(1.2),
      lengthInYInitSubmap_(1.8),
      marginInitSubmap_(0.3),
      initSubmapHeightOffset_(0.0)
{
#ifndef NDEBUG
  // Print a warning if built in debug.
  RCLCPP_WARN(nodeHandle_->get_logger(), "CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  RCLCPP_INFO(nodeHandle_->get_logger(), "Elevation mapping node started.");

  readParameters();
  setupSubscribers();
  setupServices();
  setupTimers();

  transformBuffer_ = std::make_shared<tf2_ros::Buffer>(nodeHandle_->get_clock());
  transformListener_ = std::make_shared<tf2_ros::TransformListener>(*transformBuffer_);
  obstaclesGridPub_ = nodeHandle_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "obstacles_grid", rclcpp::QoS(1).transient_local());
  occupancyPub_ = nodeHandle_->create_publisher<nav_msgs::msg::OccupancyGrid>(
  "occupancy", rclcpp::QoS(1).transient_local());
  debugMarkersPub_ = nodeHandle_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "cap_debug_markers", rclcpp::QoS(1).transient_local());
  initialize();

  RCLCPP_INFO(nodeHandle_->get_logger(), "Successfully launched node.");
}

void ElevationMapping::setupSubscribers() {  // Handle deprecated point_cloud_topic and input_sources configuration.
  auto res = nodeHandle_->get_topic_names_and_types();
  for (auto a:res){
    RCLCPP_INFO(nodeHandle_->get_logger(), "topic: %s", a.first.c_str());
  }

  const bool configuredInputSources = inputSources_.configureFromRos("input_sources");
  const bool hasDeprecatedPointcloudTopic = nodeHandle_->get_parameter("point_cloud_topic", pointCloudTopic_);
  if (hasDeprecatedPointcloudTopic) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Parameter 'point_cloud_topic' is deprecated, please use 'input_sources' instead.");
  }
  /*if (!configuredInputSources && hasDeprecatedPointcloudTopic) {
    pointCloudSubscriber_ = nodeHandle_->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointCloudTopic_, 1, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          pointCloudCallback(msg, true, sensorProcessor_);
        });
  }*/
  if (configuredInputSources) {
    inputSources_.registerCallbacks(*this, std::make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
    // inputSources_.registerCallbacks(*this, std::make_pair("pointcloud", pointCloudCallback));
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Input sources not configured!");
  }

  if (!robotPoseTopic_.empty()) {
    robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_, qos_profile);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  } else {
    ignoreRobotMotionUpdates_ = true;
  }
}

void ElevationMapping::setupServices() {
  // Multi-threading for fusion.
  fusionTriggerService_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "trigger_fusion",
      std::bind(&ElevationMapping::fuseEntireMapServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      fusionServiceGroup_);

  fusedSubmapService_ = nodeHandle_->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_submap",
      std::bind(&ElevationMapping::getFusedSubmapServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      fusionServiceGroup_);

  rawSubmapService_ = nodeHandle_->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_raw_submap",
      std::bind(&ElevationMapping::getRawSubmapServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      fusionServiceGroup_);

  clearMapService_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "clear_map",
      std::bind(&ElevationMapping::clearMapServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  enableUpdatesService_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "enable_updates",
      std::bind(&ElevationMapping::enableUpdatesServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  disableUpdatesService_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "disable_updates",
      std::bind(&ElevationMapping::disableUpdatesServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  maskedReplaceService_ = nodeHandle_->create_service<grid_map_msgs::srv::SetGridMap>(
      "masked_replace",
      std::bind(&ElevationMapping::maskedReplaceServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  saveMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>(
      "save_map",
      std::bind(&ElevationMapping::saveMapServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  loadMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>(
      "load_map",
      std::bind(&ElevationMapping::loadMapServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

void ElevationMapping::setupTimers() {
  // TODO: mapUpdateTimer_ is originally single shot and autostart is set to false
  // mapUpdateTimer_ = rclcpp::create_timer(nodeHandle_, nodeHandle_->get_clock(),
  //                                        maxNoUpdateDuration_,
  //                                        &ElevationMapping::mapUpdateTimerCallback);

  if (fusedMapPublishTimerDuration_.seconds() != 0.0) {
    fusedMapPublishTimer_ = rclcpp::create_timer(
      nodeHandle_, nodeHandle_->get_clock(), fusedMapPublishTimerDuration_,
      std::bind(&ElevationMapping::publishFusedMapCallback, this));
  }

  // Multi-threading for visibility cleanup. Visibility clean-up does not help when continuous clean-up is enabled.
  if (map_.enableVisibilityCleanup_ &&
      (visibilityCleanupTimerDuration_.seconds() != 0.0) &&
      !map_.enableContinuousCleanup_) {
    visibilityCleanupTimer_ = rclcpp::create_timer(
      nodeHandle_, nodeHandle_->get_clock(), visibilityCleanupTimerDuration_,
      std::bind(&ElevationMapping::visibilityCleanupCallback, this));
  }
}

ElevationMapping::~ElevationMapping() {
  // Shutdown all services/timers safely.
  {
    rawSubmapService_.reset();
    fusionTriggerService_.reset();
    fusedSubmapService_.reset();
    if (fusedMapPublishTimer_) fusedMapPublishTimer_->cancel();
  }
  {
    if (visibilityCleanupTimer_) visibilityCleanupTimer_->cancel();
  }

  // NO rclcpp::shutdown() aquí (lo hace el main/launch).
  // Join threads (si se reactivan en el futuro).
  /*if (fusionServiceThread_.joinable()) fusionServiceThread_.join();
    if (visibilityCleanupThread_.joinable()) visibilityCleanupThread_.join();*/
}

bool ElevationMapping::readParameters() {
  // ElevationMapping parameters.
  nodeHandle_->declare_parameter("point_cloud_topic","");
  //FIXME: Fix for case when robot pose is not defined
  nodeHandle_->declare_parameter("robot_pose_with_covariance_topic", std::string("/pose"));
  nodeHandle_->declare_parameter("track_point_frame_id", std::string("/robot"));
  nodeHandle_->declare_parameter("track_point_x", 0.0);
  nodeHandle_->declare_parameter("track_point_y", 0.0);
  nodeHandle_->declare_parameter("track_point_z", 0.0);
  nodeHandle_->declare_parameter("robot_pose_cache_size", 200);

  // nodeHandle_->get_parameter("point_cloud_topic", pointCloudTopic_);
  nodeHandle_->get_parameter("robot_pose_with_covariance_topic", robotPoseTopic_);
  nodeHandle_->get_parameter("track_point_frame_id", trackPointFrameId_);
  nodeHandle_->get_parameter("track_point_x", trackPoint_.x());
  nodeHandle_->get_parameter("track_point_y", trackPoint_.y());
  nodeHandle_->get_parameter("track_point_z", trackPoint_.z());
  nodeHandle_->get_parameter("robot_pose_cache_size", robotPoseCacheSize_);

  // --- layers.* (declaración y lectura) ---
  nodeHandle_->declare_parameter("layers.rough_window_m",        layers_.rough_window_m);
  nodeHandle_->declare_parameter("layers.step_window_m",         layers_.step_window_m);
  nodeHandle_->declare_parameter("layers.slope_long_max_rad",    layers_.slope_long_max_rad);
  nodeHandle_->declare_parameter("layers.slope_lat_max_rad",     layers_.slope_lat_max_rad);
  nodeHandle_->declare_parameter("layers.step_max_m",            layers_.step_max_m);
  nodeHandle_->declare_parameter("layers.clearance_min_m",       layers_.clearance_min_m);
  nodeHandle_->declare_parameter("layers.footprint_radius_m",    layers_.footprint_radius_m);
  nodeHandle_->declare_parameter("layers.clearance_nominal_m",   layers_.clearance_nominal_m);
  nodeHandle_->declare_parameter("layers.trav_slope_max_rad",    layers_.trav_slope_max_rad);
  nodeHandle_->declare_parameter("layers.trav_rough_max_m",      layers_.trav_rough_max_m);
  nodeHandle_->declare_parameter("layers.trav_w_slope",          layers_.trav_w_slope);
  nodeHandle_->declare_parameter("layers.trav_w_rough",          layers_.trav_w_rough);
  nodeHandle_->declare_parameter("layers.cvar_alpha",            layers_.cvar_alpha);
  nodeHandle_->declare_parameter("layers.cvar_window_m",         layers_.cvar_window_m);

  nodeHandle_->get_parameter("layers.rough_window_m",        layers_.rough_window_m);
  nodeHandle_->get_parameter("layers.step_window_m",         layers_.step_window_m);
  nodeHandle_->get_parameter("layers.slope_long_max_rad",    layers_.slope_long_max_rad);
  nodeHandle_->get_parameter("layers.slope_lat_max_rad",     layers_.slope_lat_max_rad);
  nodeHandle_->get_parameter("layers.step_max_m",            layers_.step_max_m);
  nodeHandle_->get_parameter("layers.clearance_min_m",       layers_.clearance_min_m);
  nodeHandle_->get_parameter("layers.footprint_radius_m",    layers_.footprint_radius_m);
  nodeHandle_->get_parameter("layers.clearance_nominal_m",   layers_.clearance_nominal_m);
  nodeHandle_->get_parameter("layers.trav_slope_max_rad",    layers_.trav_slope_max_rad);
  nodeHandle_->get_parameter("layers.trav_rough_max_m",      layers_.trav_rough_max_m);
  nodeHandle_->get_parameter("layers.trav_w_slope",          layers_.trav_w_slope);
  nodeHandle_->get_parameter("layers.trav_w_rough",          layers_.trav_w_rough);
  nodeHandle_->get_parameter("layers.cvar_alpha",            layers_.cvar_alpha);
  nodeHandle_->get_parameter("layers.cvar_window_m",         layers_.cvar_window_m);

  // Toggles
  nodeHandle_->declare_parameter("layers_enable.slope", true);
  nodeHandle_->declare_parameter("layers_enable.rough", true);
  nodeHandle_->declare_parameter("layers_enable.step", true);
  nodeHandle_->declare_parameter("layers_enable.obstacles_binary", true);
  nodeHandle_->declare_parameter("layers_enable.negative", true);
  nodeHandle_->declare_parameter("layers_enable.cvar", true);

  nodeHandle_->get_parameter("layers_enable.slope", enable_.slope);
  nodeHandle_->get_parameter("layers_enable.rough", enable_.rough);
  nodeHandle_->get_parameter("layers_enable.step", enable_.step);
  nodeHandle_->get_parameter("layers_enable.obstacles_binary", enable_.obstacles_binary);
  nodeHandle_->get_parameter("layers_enable.negative", enable_.negative);
  nodeHandle_->get_parameter("layers_enable.cvar", enable_.cvar);

  // Rates / thresholds
  nodeHandle_->declare_parameter("heavy_every_n", 3);
  nodeHandle_->declare_parameter("cvar_tau", 0.70);
  nodeHandle_->get_parameter("heavy_every_n", rates_.heavy_every_n);
  nodeHandle_->get_parameter("cvar_tau", rates_.cvar_tau);

  // Negative robusto
  nodeHandle_->declare_parameter("negative_min_valid", 0.50);
  nodeHandle_->declare_parameter("negative_slope_gate", 0.35);
  nodeHandle_->declare_parameter("negative_drop_thresh_m", 0.25);
  nodeHandle_->declare_parameter("negative_ring_m", 0.40);

  nodeHandle_->get_parameter("negative_min_valid",  rates_.neg_min_valid_ratio);
  nodeHandle_->get_parameter("negative_slope_gate", rates_.neg_slope_gate_rad);
  nodeHandle_->get_parameter("negative_drop_thresh_m", rates_.neg_drop_thresh_m);
  nodeHandle_->get_parameter("negative_ring_m", rates_.neg_ring_m);

  nodeHandle_->declare_parameter("layers_enable.grid",            true);
  nodeHandle_->declare_parameter("layers_enable.frontier",        true);
  nodeHandle_->declare_parameter("layers_enable.multi_cost",      true);
  nodeHandle_->declare_parameter("layers_enable.no_go",           true);
  nodeHandle_->declare_parameter("layers_enable.frontier_filter", true);
  nodeHandle_->declare_parameter("layers_enable.occupancy_like",  true);

  nodeHandle_->get_parameter("layers_enable.grid",            enable_.grid);
  nodeHandle_->get_parameter("layers_enable.frontier",        enable_.frontier);
  nodeHandle_->get_parameter("layers_enable.multi_cost",      enable_.multi_cost);
  nodeHandle_->get_parameter("layers_enable.no_go",           enable_.no_go);
  nodeHandle_->get_parameter("layers_enable.frontier_filter", enable_.frontier_filter);
  nodeHandle_->get_parameter("layers_enable.occupancy_like",  enable_.occupancy_like);

  // --- params de capas nuevas ---
  nodeHandle_->declare_parameter("grid.var_thresh",            0.05);   // m^2
  nodeHandle_->declare_parameter("grid.gate_by_variance",      true);
  nodeHandle_->declare_parameter("frontier.edge_is_unknown",   true);
  nodeHandle_->declare_parameter("multi_cost.tau",             0.60);   // umbral de no_go sobre coste combinado
  nodeHandle_->declare_parameter("no_go.inflate_m",            0.20);   // m (margen)
  nodeHandle_->declare_parameter("frontier.clearance_m",       0.10);   // m (separación de no_go)

  nodeHandle_->get_parameter("grid.var_thresh",           layers_.grid_var_thresh);
  nodeHandle_->get_parameter("grid.gate_by_variance",     layers_.grid_gate_by_variance);
  nodeHandle_->get_parameter("frontier.edge_is_unknown",  layers_.frontier_edge_is_unknown);
  nodeHandle_->get_parameter("multi_cost.tau",            layers_.multi_cost_tau);
  nodeHandle_->get_parameter("no_go.inflate_m",           layers_.no_go_inflate_m);
  nodeHandle_->get_parameter("frontier.clearance_m",      layers_.frontier_clearance_m);
  
  assert(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  nodeHandle_->declare_parameter("min_update_rate", 2.0);
  nodeHandle_->get_parameter("min_update_rate", minUpdateRate);
  if (minUpdateRate == 0.0) {
    maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(0);
    RCLCPP_WARN(nodeHandle_->get_logger(), "Rate for publishing the map is zero.");
  } else {
    maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(1.0 / minUpdateRate);
  }
  assert(maxNoUpdateDuration_.seconds() != 0.0);

  double timeTolerance;
  nodeHandle_->declare_parameter("time_tolerance", 0.0);
  nodeHandle_->get_parameter("time_tolerance", timeTolerance);
  timeTolerance_ = rclcpp::Duration::from_seconds(timeTolerance);

  double fusedMapPublishingRate;
  nodeHandle_->declare_parameter("fused_map_publishing_rate", 1.0);
  nodeHandle_->get_parameter("fused_map_publishing_rate", fusedMapPublishingRate);
  if (fusedMapPublishingRate == 0.0) {
    fusedMapPublishTimerDuration_ = rclcpp::Duration::from_seconds(0.0);
    RCLCPP_WARN(nodeHandle_->get_logger(),
        "Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is called.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    isContinuouslyFusing_ = true;
    fusedMapPublishTimerDuration_ = rclcpp::Duration::from_seconds(0.0);
  } else {
    fusedMapPublishTimerDuration_ = rclcpp::Duration::from_seconds(1.0 / fusedMapPublishingRate);
  }

  double visibilityCleanupRate;
  nodeHandle_->declare_parameter("visibility_cleanup_rate", 1.0);
  nodeHandle_->get_parameter("visibility_cleanup_rate", visibilityCleanupRate);
  if (visibilityCleanupRate == 0.0) {
    visibilityCleanupTimerDuration_ = rclcpp::Duration::from_seconds(0.0);
    RCLCPP_WARN(nodeHandle_->get_logger(), "Rate for visibility cleanup is zero and therefore disabled.");
  } else {
    visibilityCleanupTimerDuration_ = rclcpp::Duration::from_seconds(1.0 / visibilityCleanupRate);
    map_.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  // ElevationMap parameters. TODO Move this to the elevation map class.
  nodeHandle_->declare_parameter("map_frame_id", std::string("/map"));
  nodeHandle_->get_parameter("map_frame_id", mapFrameId_);
  map_.setFrameId(mapFrameId_);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;

  nodeHandle_->declare_parameter("length_in_x", 1.5);
  nodeHandle_->declare_parameter("length_in_y", 1.5);
  nodeHandle_->declare_parameter("position_x",  0.0);
  nodeHandle_->declare_parameter("position_y",  0.0);
  nodeHandle_->declare_parameter("resolution",  0.01);

  nodeHandle_->get_parameter("length_in_x", length(0));
  nodeHandle_->get_parameter("length_in_y", length(1));
  nodeHandle_->get_parameter("position_x", position.x());
  nodeHandle_->get_parameter("position_y", position.y());
  nodeHandle_->get_parameter("resolution", resolution);
  map_.setGeometry(length, resolution, position);

  nodeHandle_->declare_parameter("min_variance", pow(0.003, 2));
  nodeHandle_->declare_parameter("max_variance", pow(0.03, 2));
  nodeHandle_->declare_parameter("mahalanobis_distance_threshold", 2.5);
  nodeHandle_->declare_parameter("multi_height_noise", pow(0.003, 2));
  nodeHandle_->declare_parameter("min_horizontal_variance", pow(resolution / 2.0, 2));  // two-sigma
  nodeHandle_->declare_parameter("max_horizontal_variance", 0.5);
  nodeHandle_->declare_parameter("underlying_map_topic", std::string());
  nodeHandle_->declare_parameter("enable_visibility_cleanup", true);
  nodeHandle_->declare_parameter("enable_continuous_cleanup", false);
  nodeHandle_->declare_parameter("scanning_duration", 1.0);
  nodeHandle_->declare_parameter("masked_replace_service_mask_layer_name", std::string("mask"));

  nodeHandle_->get_parameter("min_variance", map_.minVariance_);
  nodeHandle_->get_parameter("max_variance", map_.maxVariance_);
  nodeHandle_->get_parameter("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_);
  nodeHandle_->get_parameter("multi_height_noise", map_.multiHeightNoise_);
  nodeHandle_->get_parameter("min_horizontal_variance", map_.minHorizontalVariance_);  // two-sigma
  nodeHandle_->get_parameter("max_horizontal_variance", map_.maxHorizontalVariance_);
  nodeHandle_->get_parameter("underlying_map_topic", map_.underlyingMapTopic_);
  nodeHandle_->get_parameter("enable_visibility_cleanup", map_.enableVisibilityCleanup_);
  nodeHandle_->get_parameter("enable_continuous_cleanup", map_.enableContinuousCleanup_);
  nodeHandle_->get_parameter("scanning_duration", map_.scanningDuration_);
  nodeHandle_->get_parameter("masked_replace_service_mask_layer_name", maskedReplaceServiceMaskLayerName_);

  // Settings for initializing elevation map
  nodeHandle_->declare_parameter("initialize_elevation_map", false);
  nodeHandle_->declare_parameter("initialization_method", 0);
  nodeHandle_->declare_parameter("length_in_x_init_submap", 1.2);
  nodeHandle_->declare_parameter("length_in_y_init_submap", 1.8);
  nodeHandle_->declare_parameter("margin_init_submap", 0.3);
  nodeHandle_->declare_parameter("init_submap_height_offset", 0.0);
  nodeHandle_->declare_parameter("target_frame_init_submap", std::string("/footprint"));

  nodeHandle_->get_parameter("initialize_elevation_map", initializeElevationMap_);
  nodeHandle_->get_parameter("initialization_method", initializationMethod_);
  nodeHandle_->get_parameter("length_in_x_init_submap", lengthInXInitSubmap_);
  nodeHandle_->get_parameter("length_in_y_init_submap", lengthInYInitSubmap_);
  nodeHandle_->get_parameter("margin_init_submap", marginInitSubmap_);
  nodeHandle_->get_parameter("init_submap_height_offset", initSubmapHeightOffset_);
  nodeHandle_->get_parameter("target_frame_init_submap", targetFrameInitSubmap_);

  // No-go parameters
  nodeHandle_->declare_parameter("no_go.use_multi_cost",       false); // <- apagamos el modo antiguo
  nodeHandle_->declare_parameter("no_go.slope_blocking_rad",   0.85);  // ~31.5°
  nodeHandle_->declare_parameter("no_go.rough_blocking_m",     0.15);  // 8 cm
  nodeHandle_->declare_parameter("no_go.use_obstacles",        false);
  nodeHandle_->declare_parameter("no_go.use_negatives",        true);
  nodeHandle_->declare_parameter("no_go.use_cvar",             true);
  nodeHandle_->declare_parameter("no_go.cvar_tau",             0.90);

  nodeHandle_->get_parameter("no_go.use_multi_cost",     layers_.no_go_use_multi_cost);
  nodeHandle_->get_parameter("no_go.slope_blocking_rad", layers_.no_go_slope_blocking_rad);
  nodeHandle_->get_parameter("no_go.rough_blocking_m",   layers_.no_go_rough_blocking_m);
  nodeHandle_->get_parameter("no_go.use_obstacles",      layers_.no_go_use_obstacles);
  nodeHandle_->get_parameter("no_go.use_negatives",      layers_.no_go_use_negatives);
  nodeHandle_->get_parameter("no_go.use_cvar",           layers_.no_go_use_cvar);
  nodeHandle_->get_parameter("no_go.cvar_tau",           layers_.no_go_cvar_tau);



  nodeHandle_->declare_parameter("robot_base_frame_id", std::string("/robot"));

  // SensorProcessor parameters. Deprecated, use the sensorProcessor from within input sources instead!
  /*std::string sensorType;
  nodeHandle_->declare_parameter("sensor_processor/type", std::string("structured_light"));
  nodeHandle_->get_parameter("sensor_processor/type", sensorType);

  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{
      nodeHandle_->get_parameter("robot_base_frame_id").as_string(), mapFrameId_};
  if (sensorType == "structured_light") {
    sensorProcessor_.reset(new StructuredLightSensorProcessor(nodeHandle_, generalSensorProcessorConfig)); // ERROR
  } else if (sensorType == "stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(nodeHandle_, generalSensorProcessorConfig));
  } else if (sensorType == "laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(nodeHandle_, generalSensorProcessorConfig));
  } else if (sensorType == "perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(nodeHandle_, generalSensorProcessorConfig));
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "The sensor type %s is not available.", sensorType.c_str());
  }
  if (!sensorProcessor_->readParameters()) {
    return false;
  }*/
  if (!robotMotionMapUpdater_.readParameters()) {
    return false;
  }

  return true;
}

bool ElevationMapping::initialize() {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Elevation mapping node initializing ... ");

  fusionServiceGroup_ = nodeHandle_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  //fusionServiceThread_ = std::thread(boost::bind(&ElevationMapping::runFusionServiceThread, this));
  rclcpp::sleep_for(std::chrono::seconds(1));  // Need this to get the TF caches fill up.
  // resetMapUpdateTimer();
  if (fusedMapPublishTimer_) fusedMapPublishTimer_->reset();
  // visibilityCleanupThread_ = boost::thread(boost::bind(&ElevationMapping::visibilityCleanupThread, this));
  // visibilityCleanupTimer_.reset();  //TODO:foxy does not have timer autostart flag implemented, fix in future version
  initializeElevationMap();
  return true;
}

/*void ElevationMapping::runFusionServiceThread() {
  rclcpp::Rate loopRate(20);

  while (rclcpp::ok()) {
    fusionServiceQueue_.callAvailable();
    loopRate.sleep();
  }
}*/

/*void ElevationMapping::visibilityCleanupThread() {
  rclcpp::Rate loopRate(20);

  while (rclcpp::ok()) {
    visibilityCleanupQueue_.callAvailable();
    loopRate.sleep();
  }
}*/

void ElevationMapping::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointCloudMsg,
                                          bool publishPointCloud,
                                          const SensorProcessorBase::Ptr& sensorProcessor_) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
  if (!updatesEnabled_) {
    auto clock = nodeHandle_->get_clock();
    RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), *(clock), 10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    if (publishPointCloud) {
      map_.setTimestamp(nodeHandle_->get_clock()->now());
      map_.postprocessAndPublishRawElevationMap();
    }
    return;
  }

  // Check if point cloud has corresponding robot pose at the beginning
  if (!receivedFirstMatchingPointcloudAndPose_) {
    const double oldestPoseTime = robotPoseCache_.getOldestTime().seconds();
    const double currentPointCloudTime = rclcpp::Time(pointCloudMsg->header.stamp, RCL_ROS_TIME).seconds();

    if (currentPointCloudTime < oldestPoseTime) {
      auto clock = nodeHandle_->get_clock();
      RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), *(clock), 5,
                           "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
      return;
    } else {
      RCLCPP_INFO(nodeHandle_->get_logger(), "First corresponding point cloud and pose found, elevation mapping started. ");
      receivedFirstMatchingPointcloudAndPose_ = true;
    }
  }

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  // Use the ROS message timestamp for pose lookup. The PCL header stamp can be
  // in different units depending on conversion path, which breaks matching
  // pointclouds to odometry in the cache.
  lastPointCloudUpdateTime_ = rclcpp::Time(pointCloudMsg->header.stamp, RCL_ROS_TIME);

  RCLCPP_DEBUG(nodeHandle_->get_logger(), "ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!ignoreRobotMotionUpdates_) {
    std::shared_ptr<const nav_msgs::msg::Odometry> poseMessage = robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
      if (robotPoseCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds()) {
        RCLCPP_ERROR(nodeHandle_->get_logger(), "The oldest pose available is at %f, requested pose at %f",
                     robotPoseCache_.getOldestTime().seconds(), lastPointCloudUpdateTime_.seconds());
      } else {
        RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?",
                     lastPointCloudUpdateTime_.seconds());
      }
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  RCLCPP_INFO_STREAM(nodeHandle_->get_logger(), "Variance is:" << robotPoseCovariance);
  PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
  Eigen::VectorXf measurementVariances;
  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                 pointCloudMsg->header.frame_id)) {
    if (!sensorProcessor_->isTfAvailableInBuffer()) {
      rclcpp::Clock clock;
      RCLCPP_INFO_THROTTLE(nodeHandle_->get_logger(), clock, 10,
                           "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
      return;
    }
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Point cloud could not be processed.");
    return;
  }

  // === Log robusto de variancias ===
  if (measurementVariances.size() == 0) {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Variance vector is empty.");
  } else {
    const size_t N = measurementVariances.size();
    std::array<size_t, 5> picks{0, N/4, N/2, (3*N)/4, N-1};
    std::ostringstream os;
    os << "Variance samples (" << N << "): ";
    for (size_t k = 0; k < picks.size(); ++k) {
      size_t idx = std::min(picks[k], N-1);
      os << measurementVariances[idx];
      if (k + 1 < picks.size()) os << " , ";
    }
    RCLCPP_INFO(nodeHandle_->get_logger(), "%s", os.str().c_str());
  }

  if (measurementVariances.size() >= 121) {
    RCLCPP_INFO(nodeHandle_->get_logger(),
                "Variance[0,10,100,110,120]: %f , %f , %f , %f , %f",
                measurementVariances[0], measurementVariances[10],
                measurementVariances[100], measurementVariances[110],
                measurementVariances[120]);
  } else if (measurementVariances.size() > 0) {
    const int last = static_cast<int>(measurementVariances.size()) - 1;
    RCLCPP_INFO(nodeHandle_->get_logger(),
                "Variance[0,last]: %f , %f (N=%zu)",
                measurementVariances[0], measurementVariances[last],
                measurementVariances.size());
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Updating process noise failed.");
    return;
  }

  // Clear the map if continuous clean-up was enabled.
  if (map_.enableContinuousCleanup_) {
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Clearing elevation map before adding new point cloud.");
    map_.clear();
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Adding point cloud to elevation map failed.");
    return;
  }

  // ====== CAPAS LIGERAS + PESADAS (con protección) ======
  try {
    frameCount_++;

    const double cell = map_.getRawGridMap().getResolution();

    // BASE (barato, cada nube)
    if (enable_.slope) em::layers::addSlope     (map_.getRawGridMap(), cell);
    if (enable_.rough) em::layers::addRoughness (map_.getRawGridMap(), layers_.rough_window_m);
    if (enable_.step)  em::layers::addStep      (map_.getRawGridMap(), layers_.step_window_m);

    // --- GRID + FRONTIER ---
    if (enable_.grid) {
      em::layers::addGridKnown(map_.getRawGridMap(),
                              layers_.grid_gate_by_variance,
                              layers_.grid_var_thresh,
                              "elevation", "variance", "grid");
    }
    if (enable_.frontier) {
      em::layers::addFrontierFromGrid(map_.getRawGridMap(),
                                      "grid", "frontier",
                                      layers_.frontier_edge_is_unknown);
    }

    // --- COSTE COMBINADO (slope/rough/cvar/negatives) ---
    if (enable_.multi_cost) {
      std::vector<std::string> cost_layers = {"slope", "rough", "cvar_risk", "negatives"};
      std::vector<double>      w           = {layers_.trav_w_slope, layers_.trav_w_rough, 0.2, 0.3};
      std::vector<std::pair<double,double>> minmax = {
        {0.0, layers_.trav_slope_max_rad},  // slope (rad) normaliza contra tu máximo deseable
        {0.0, layers_.trav_rough_max_m},    // rough (m)
        {0.0, 1.0},                         // cvar_risk ya [0,1]
        {0.0, 1.0}                          // negatives 0/1
      };
      em::layers::addMultiCost(map_.getRawGridMap(), cost_layers, w, minmax, "multi_cost");
    }

    // --- NO-GO desde coste + filtro de frontera ---
    if (enable_.no_go) {
      em::layers::addNoGoFromCost(map_.getRawGridMap(),
                                  "multi_cost",
                                  layers_.multi_cost_tau,
                                  layers_.no_go_inflate_m,
                                  "no_go");
    }
    if (enable_.frontier_filter) {
      em::layers::filterFrontierByNoGo(map_.getRawGridMap(),
                                      "frontier", "no_go",
                                      layers_.frontier_clearance_m,
                                      "frontier_ok");
    }

    // // (Opcional) Occupancy "like" para planners 2D
    // if (enable_.occupancy_like) {
    //   // usa "no_go" como "ocupado", y "grid" para known/unknown
    //   em::layers::addOccupancyLike(map_.getRawGridMap(),
    //                               "grid", "no_go", "occupancy_like");
    //   em::layers::occupancyLikeToMask(map_.getRawGridMap(),
    //                             "occupancy_like", "occupancy_mask");
    // }

    // if (occupancyPub_ && occupancyPub_->get_subscription_count() > 0 &&
    //     map_.getRawGridMap().exists("occupancy_mask")) {
    //   nav_msgs::msg::OccupancyGrid og;
    //   grid_map::GridMapRosConverter::toOccupancyGrid(
    //       map_.getRawGridMap(), "occupancy_mask", 0.0, 1.0, og);
    //   og.header.stamp    = nodeHandle_->get_clock()->now();
    //   og.header.frame_id = map_.getFrameId();
    //   occupancyPub_->publish(og);
    // }

    // if (enable_.no_go) {
    //   if (layers_.no_go_use_multi_cost) {
    //     // MODO ANTIGUO (por si quieres dejarlo conmutado)
    //     em::layers::addNoGoFromCost(map_.getRawGridMap(),
    //                                 "multi_cost",
    //                                 layers_.multi_cost_tau,
    //                                 layers_.no_go_inflate_m,
    //                                 "no_go");
    //   } else {
    //     // NUEVO VETO DURO, SIN PENALIZAR CUESTAS TRAVESABLES
    //     em::layers::addNoGoHard(map_.getRawGridMap(),
    //                             layers_.no_go_slope_blocking_rad,
    //                             layers_.no_go_rough_blocking_m,
    //                             layers_.no_go_use_obstacles,
    //                             layers_.no_go_use_negatives,
    //                             layers_.no_go_use_cvar,
    //                             layers_.no_go_cvar_tau,
    //                             layers_.no_go_inflate_m,
    //                             "no_go");
    //   }
    // }

    // Obstáculo binario geométrico (pendiente + escalón)
    if (enable_.obstacles_binary) {
      em::layers::addObstacleBinaryFromGeom(
          map_.getRawGridMap(),
          /*slope_thresh_rad=*/layers_.slope_long_max_rad,
          /*step_thresh_m=*/   layers_.step_max_m);
    }

    // NEGATIVE y CVaR (más caro) — cada N nubes
    if (frameCount_ % std::max(1, rates_.heavy_every_n) == 0) {
      if (enable_.negative) {
        em::layers::addNegativeObstaclesRobust(
            map_.getRawGridMap(),
            /*drop_thresh_m=*/ rates_.neg_drop_thresh_m,
            /*ring_m=*/        rates_.neg_ring_m,
            /*min_valid=*/     rates_.neg_min_valid_ratio,
            /*slope_gate=*/    rates_.neg_slope_gate_rad,
            /*elev=*/          "elevation",
            /*slope=*/         "slope",
            /*out=*/           "negatives");
      }
      if (enable_.cvar) {
        // addCvarTraversability escribe en "cvar_risk"
        em::layers::addCvarTraversability(
            map_.getRawGridMap(),
            /*alpha=*/  layers_.cvar_alpha,
            /*win_m=*/  layers_.cvar_window_m);
      }
    }
////////////////////////////////////////////////////////////////////////////////////////////
    // ---- NAVGRID desde NEGATIVES: desconocido/libre/ocupado (solo negatives) ----

  // auto& gm = map_.getRawGridMap();

  // // Usa elevation_inpainted si existe (evita marcar desconocido por pequeños huecos)
  // const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
  // const std::string neg  = "negatives";
  // const std::string out  = "navgrid";   // capa float: NaN (desconocido), 0.0 (libre), 1.0 (ocupado)

  // if (!gm.exists(elev) || !gm.exists(neg)) {
  //   RCLCPP_WARN(nodeHandle_->get_logger(),
  //               "navgrid: faltan capas '%s' o '%s'", elev.c_str(), neg.c_str());
  // } else {
  //   if (!gm.exists(out)) gm.add(out, std::numeric_limits<float>::quiet_NaN());

  //   grid_map::Matrix& L       = gm[out];
  //   const grid_map::Matrix& E = gm[elev];
  //   const grid_map::Matrix& N = gm[neg];

  //   const float tau = 0.50f;  // negatives > tau => OCUPADO. Ajusta a tu gusto.

  //   for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
  //     const grid_map::Index idx(*it);
  //     const float e = E(idx(0), idx(1));
  //     if (std::isnan(e)) {
  //       // no observado => desconocido
  //       L(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
  //     } else {
  //       const float nv = N(idx(0), idx(1));
  //       L(idx(0), idx(1)) = (nv > tau) ? 1.0f : 0.0f;  // 1=ocupado, 0=libre
  //     }
  //   }

  //   // Publica OccupancyGrid desde 'navgrid': 0->0, 1->100, NaN->-1
  //   if (occupancyPub_ && occupancyPub_->get_subscription_count() > 0) {
  //     nav_msgs::msg::OccupancyGrid og;
  //     grid_map::GridMapRosConverter::toOccupancyGrid(gm, out, 0.0f, 1.0f, og);
  //     og.header.stamp    = nodeHandle_->get_clock()->now();
  //     og.header.frame_id = gm.getFrameId();
  //     occupancyPub_->publish(og);
  //   }
  // }
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
//// nagrid con constantes
{
auto& gm = map_.getRawGridMap();

  // Parámetros del robot (ajústalos a tu vehículo)
  const double mu        = 0.62;  // coef. fricción suelo-rueda
  const double h_cg      = 0.70;  // m, altura del centro de masas
  const double track     = 1.26;  // m, ancho de vía (rueda izq a der)
  const double step_max  = 400.15;  // m, escalón superable
  const double rough_max = 0.15;  // m RMS permisible
  const double safety    = 0.035;  // margen (rad) para ir conservador

  // Límites derivados (conservadores si no separas pitch/roll)
  const double theta_trac = std::atan(mu) - safety;                 // ascenso por tracción
  const double phi_roll   = std::atan(0.5 * track / h_cg) - safety; // vuelco lateral
  const double slope_lim  = std::min(theta_trac, phi_roll);

  // Capas requeridas
  if (!gm.exists("slope") || !gm.exists("rough") || !gm.exists("step") || !gm.exists("negatives")) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "no_go_cap: faltan capas base (slope/rough/step/negatives).");
  } else {
    const std::string out_cap = "no_go_cap";
    if (!gm.exists(out_cap)) gm.add(out_cap, 0.0);

    auto& CAP   = gm[out_cap];
    const auto& S   = gm["slope"];                        // rad
    const auto& R   = gm["rough"];                        // m
    const auto& ST  = gm["step"];                         // m
    const auto& NEG = gm["negatives"];                    // [0,1]
    const auto* RISKp = gm.exists("cvar_risk") ? &gm["cvar_risk"] : nullptr;

    // Usamos elevación (inpainted si existe) para distinguir conocido/desconocido
    const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
    const auto& E = gm[elev];

    for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);

      // Desconocido => deja NaN (para que navgrid ponga -1)
      const float e = E(idx(0), idx(1));
      if (std::isnan(e)) { 
        CAP(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN(); 
        continue; 
      }

      bool block = false;
      block |= (S (idx(0), idx(1)) > slope_lim);
      block |= (R (idx(0), idx(1)) > rough_max);
      block |= (ST(idx(0), idx(1)) > step_max);
      block |= (NEG(idx(0), idx(1)) > 0.5f);
      if (RISKp) block |= ((*RISKp)(idx(0), idx(1)) > 0.9f);

      CAP(idx(0), idx(1)) = block ? 1.0f : 0.0f;
    }
  }
  }
//NAVGRID final: desconocido/libre/ocupado usando no_go_cap
{
  // auto& gm = map_.getRawGridMap();
  // const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
  // const std::string cap  = "no_go_cap";
  // const std::string out  = "navgrid"; // NaN=desconocido, 0.0=libre, 1.0=ocupado

  // if (!gm.exists(elev) || !gm.exists(cap)) {
  //   RCLCPP_WARN(nodeHandle_->get_logger(), "navgrid: faltan capas '%s' o '%s'", elev.c_str(), cap.c_str());
  // } else {
  //   if (!gm.exists(out)) gm.add(out, std::numeric_limits<float>::quiet_NaN());

  //   auto& L       = gm[out];
  //   const auto& E = gm[elev];
  //   const auto& C = gm[cap];

  //   for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
  //     const grid_map::Index idx(*it);
  //     const float e = E(idx(0), idx(1));
  //     if (std::isnan(e)) {
  //       L(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
  //     } else {
  //       L(idx(0), idx(1)) = (C(idx(0), idx(1)) > 0.5f) ? 1.0f : 0.0f;
  //     }
  //   }

  //   // Publica OccupancyGrid (0->0, 1->100, NaN->-1)
  //   if (occupancyPub_ && occupancyPub_->get_subscription_count() > 0) {
  //     nav_msgs::msg::OccupancyGrid og;
  //     grid_map::GridMapRosConverter::toOccupancyGrid(gm, out, 0.0f, 1.0f, og);
  //     og.header.stamp    = nodeHandle_->get_clock()->now();
  //     og.header.frame_id = gm.getFrameId();
  //     occupancyPub_->publish(og);
  //   }
  // }


  // --- NAVGRID desde NEGATIVES con tau parametrizable + filtro anti-puntos ---
auto& gm = map_.getRawGridMap();
const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
const std::string neg  = "negatives";
const std::string out  = "navgrid";

double navgrid_tau = 0.65;
nodeHandle_->get_parameter("navgrid_tau", navgrid_tau);

if (gm.exists(elev) && gm.exists(neg)) {
  if (!gm.exists(out)) gm.add(out, std::numeric_limits<float>::quiet_NaN());

  grid_map::Matrix& L       = gm[out];
  const grid_map::Matrix& E = gm[elev];
  const grid_map::Matrix& N = gm[neg];

  // 1) Umbral
  for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
    const grid_map::Index idx(*it);
    const float e = E(idx(0), idx(1));
    if (std::isnan(e)) {
      L(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();  // desconocido
    } else {
      const float nv = N(idx(0), idx(1));
      L(idx(0), idx(1)) = (nv > static_cast<float>(navgrid_tau)) ? 1.0f : 0.0f;  // 1=ocupado, 0=libre
    }
  }

  // 2) Filtro 3×3 para eliminar puntos sueltos ocupados (apertura suave)
  grid_map::Matrix Lcopy = L;
  auto is_occ = [&](int x, int y)->bool {
    const float v = Lcopy(x,y);
    return (!std::isnan(v) && v > 0.5f);
  };

  const auto size = gm.getSize(); // size(0)=Nx, size(1)=Ny


  auto inBounds = [&](int x, int y) -> bool {
  return (x >= 0 && y >= 0 && x < size(0) && y < size(1));
  };

  for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
    const grid_map::Index idx(*it);
    const float v = Lcopy(idx(0), idx(1));

    if (!std::isfinite(v)) continue;  // dejamos desconocidos como están
    if (v <= 0.5f) continue;          // solo tocamos ocupados

    int occ_nb = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;

        const int nx = idx(0) + dx;
        const int ny = idx(1) + dy;
        if (!inBounds(nx, ny)) continue;       // <-- reemplaza la llamada errónea
        if (is_occ(nx, ny)) ++occ_nb;
      }
    }

    if (occ_nb <= 1) {
      // si está solo o casi solo, bórralo
      L(idx(0), idx(1)) = 0.0f;
    }
  }

  // Publicar OccupancyGrid (0->libre, 1->ocupado, NaN->desconocido)
  if (occupancyPub_ && occupancyPub_->get_subscription_count() > 0) {
    nav_msgs::msg::OccupancyGrid og;
    grid_map::GridMapRosConverter::toOccupancyGrid(gm, out, 0.0f, 1.0f, og);
    og.header.stamp    = nodeHandle_->get_clock()->now();
    og.header.frame_id = gm.getFrameId();
    occupancyPub_->publish(og);
  }
} else {
  RCLCPP_WARN(nodeHandle_->get_logger(), "navgrid: faltan capas '%s' o '%s'", elev.c_str(), neg.c_str());
}

}



//////////////
{
// ---- DEBUG: capa 'cap_debug' (motivo de bloqueo) ----
auto& gm = map_.getRawGridMap();

const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
const bool hasVar   = gm.exists("variance");
const bool hasNeg   = gm.exists("negatives");
const bool hasCVar  = gm.exists("cvar_risk");
const bool hasSlope = gm.exists("slope");
const bool hasRough = gm.exists("rough");
const bool hasStep  = gm.exists("step");

const float var_tau   = static_cast<float>(layers_.grid_var_thresh);        // p.ej. 0.02–0.03 m^2
const float neg_tau   = 0.50f;                                              // umbral para 'negatives'
const float cvar_tau  = static_cast<float>(layers_.no_go_cvar_tau);         // umbral CVaR
const float slope_blk = static_cast<float>(layers_.no_go_slope_blocking_rad);
const float rough_blk = static_cast<float>(layers_.no_go_rough_blocking_m);
const float step_blk  = static_cast<float>(layers_.step_max_m);

const std::string out_dbg = "cap_debug";
if (!gm.exists(out_dbg)) gm.add(out_dbg, std::numeric_limits<float>::quiet_NaN());
grid_map::Matrix& D = gm[out_dbg];

std::array<int, 6> hist = {0,0,0,0,0,0};  // 0..5

for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
  const grid_map::Index idx(*it);

  // 0) desconocido (sin elevación) => VAR
  const float e = gm[elev](idx(0), idx(1));
  int reason = 0;
  if (std::isnan(e)) {
    reason = 5;
  }
  // 1) varianza alta => VAR
  else if (hasVar) {
    const float v = gm["variance"](idx(0), idx(1));
    if (std::isfinite(v) && v > var_tau) {
      reason = 5;
    }
  }

  // 2) riesgos (negativos/CVaR) => RISK
  if (reason == 0) {
    bool risky = false;
    if (hasNeg)  risky = risky || (gm["negatives"](idx(0), idx(1)) > neg_tau);
    if (hasCVar) risky = risky || (gm["cvar_risk"](idx(0), idx(1))  > cvar_tau);
    if (risky) reason = 4;
  }

  // 3) geometría dura: STEP > umbral
  if (reason == 0 && hasStep) {
    if (gm["step"](idx(0), idx(1)) > step_blk) reason = 3;
  }

  // 4) ROUGH > umbral
  if (reason == 0 && hasRough) {
    if (gm["rough"](idx(0), idx(1)) > rough_blk) reason = 2;
  }

  // 5) SLOPE > umbral
  if (reason == 0 && hasSlope) {
    if (gm["slope"](idx(0), idx(1)) > slope_blk) reason = 1;
  }

  // 6) OK
  D(idx(0), idx(1)) = static_cast<float>(reason);
  if (reason >= 0 && reason < static_cast<int>(hist.size())) hist[reason]++;
}

// Log resumen (útil para ver rápido qué está bloqueando más)
const int tot = std::accumulate(hist.begin(), hist.end(), 0);
RCLCPP_INFO(nodeHandle_->get_logger(),
  "cap_debug: OK=%d SLOPE=%d ROUGH=%d STEP=%d RISK=%d VAR=%d (tot=%d)",
  hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], tot);
}
////////////////

{
  auto& gm = map_.getRawGridMap();
  if (!gm.exists("cap_debug")) { /* nada que pintar */ }
  else {
    const double res = gm.getResolution();

    // Prepara 6 marcadores (uno por motivo).
    std::array<visualization_msgs::msg::Marker, 6> ms;
    const std::array<std::array<float,4>,6> colors = {{
      {0.0f, 1.0f, 0.0f, 0.6f},  // 0 OK      -> verde
      {1.0f, 0.5f, 0.0f, 0.8f},  // 1 SLOPE   -> naranja
      {1.0f, 1.0f, 0.0f, 0.8f},  // 2 ROUGH   -> amarillo
      {1.0f, 0.0f, 0.0f, 0.8f},  // 3 STEP    -> rojo
      {1.0f, 0.0f, 1.0f, 0.8f},  // 4 RISK    -> magenta
      {0.5f, 0.5f, 0.5f, 0.8f}   // 5 VAR     -> gris
    }};
    const auto stamp = nodeHandle_->get_clock()->now();
    for (int r = 0; r < 6; ++r) {
      auto& m = ms[r];
      m.header.frame_id = gm.getFrameId();
      m.header.stamp    = stamp;
      m.ns   = "cap_debug";
      m.id   = r;
      m.type = visualization_msgs::msg::Marker::CUBE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = res; m.scale.y = res; m.scale.z = 0.01;
      m.color.r = colors[r][0]; m.color.g = colors[r][1];
      m.color.b = colors[r][2]; m.color.a = colors[r][3];
      m.pose.orientation.w = 1.0;  // identidad
    }

    // Mete cada celda en su marcador por categoría.
    const grid_map::Matrix& D = gm["cap_debug"];
    for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
      grid_map::Index idx(*it);
      float v = D(idx(0), idx(1));
      if (!std::isfinite(v)) continue;
      int reason = static_cast<int>(std::round(v));
      if (reason < 0 || reason > 5) continue;

      grid_map::Position p; gm.getPosition(*it, p);
      geometry_msgs::msg::Point q;
      q.x = p.x(); q.y = p.y(); q.z = 0.02;  // un pelín alto para que se vea
      ms[reason].points.push_back(q);
    }

    visualization_msgs::msg::MarkerArray arr;
    for (auto& m : ms) arr.markers.push_back(m);
    debugMarkersPub_->publish(arr);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////


    // // Combinar todo a una capa final para el planner
    // em::layers::combineToFinalObstacles(
    //     map_.getRawGridMap(),
    //     /*obstacles_layer=*/ "obstacles",
    //     /*negatives_layer=*/ "negatives",
    //     /*cvar_layer=*/      "cvar_risk",
    //     /*cvar_tau=*/        rates_.cvar_tau,
    //     /*out_layer=*/       "obstacles_final");

    // (Opcional) publicar OccupancyGrid de la capa final — desactivado mientras depuramos
    // if (obstaclesGridPub_ && obstaclesGridPub_->get_subscription_count() > 0 &&
    //     map_.getRawGridMap().exists("obstacles_final")) {
    //   nav_msgs::msg::OccupancyGrid og;
    //   if (grid_map::GridMapRosConverter::toOccupancyGrid(
    //         map_.getRawGridMap(), "obstacles_final", 0.0f, 1.0f, og)) {
    //     og.header.stamp    = nodeHandle_->get_clock()->now();
    //     og.header.frame_id = map_.getFrameId();
    //     obstaclesGridPub_->publish(og);
    //   }
    // }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Layer block exception: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Layer block crashed (unknown).");
  }

  if (publishPointCloud) {
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Publishing pcl.");
    // Publish elevation map.
    map_.postprocessAndPublishRawElevationMap();
    if (isFusingEnabled()) {
      map_.fuseAll();
      map_.publishFusedElevationMap();
    }
  }
}

void ElevationMapping::mapUpdateTimerCallback() {
  if (!updatesEnabled_) {
    rclcpp::Clock clock;
    RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), clock, 10,
                         "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    map_.setTimestamp(nodeHandle_->get_clock()->now());
    map_.postprocessAndPublishRawElevationMap();
    return;
  }

  const rclcpp::Time now = nodeHandle_->get_clock()->now();
  // Si hubo nube reciente (now - lastPointCloudUpdateTime_ <= maxNoUpdateDuration_), no fuerces actualización
  if ((now - lastPointCloudUpdateTime_) <= maxNoUpdateDuration_) {
    return;
  }
  rclcpp::Clock clock;
  RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), clock, 5,
                       "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  // Update map from motion prediction.
  if (!updatePrediction(now)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Updating process noise failed.");
    return;
  }

  static int tickCount_ = 0;
  tickCount_++;

  const double cell = map_.getRawGridMap().getResolution();

  // Capas base (baratas)
  if (enable_.slope) em::layers::addSlope     (map_.getRawGridMap(), cell);
  if (enable_.rough) em::layers::addRoughness (map_.getRawGridMap(), layers_.rough_window_m);
  if (enable_.step)  em::layers::addStep      (map_.getRawGridMap(), layers_.step_window_m);

  // --- GRID + FRONTIER ---
  if (enable_.grid) {
    em::layers::addGridKnown(map_.getRawGridMap(),
                            layers_.grid_gate_by_variance,
                            layers_.grid_var_thresh,
                            "elevation", "variance", "grid");
  }
  if (enable_.frontier) {
    em::layers::addFrontierFromGrid(map_.getRawGridMap(),
                                    "grid", "frontier",
                                    layers_.frontier_edge_is_unknown);
  }

  // --- COSTE COMBINADO (slope/rough/cvar/negatives) ---
  if (enable_.multi_cost) {
    std::vector<std::string> cost_layers = {"slope", "rough", "cvar_risk", "negatives"};
    std::vector<double>      w           = {layers_.trav_w_slope, layers_.trav_w_rough, 0.2, 0.3};
    std::vector<std::pair<double,double>> minmax = {
      {0.0, layers_.trav_slope_max_rad},  // slope (rad) normaliza contra tu máximo deseable
      {0.0, layers_.trav_rough_max_m},    // rough (m)
      {0.0, 1.0},                         // cvar_risk ya [0,1]
      {0.0, 1.0}                          // negatives 0/1
    };
    em::layers::addMultiCost(map_.getRawGridMap(), cost_layers, w, minmax, "multi_cost");
  }

  // --- NO-GO desde coste + filtro de frontera ---
  if (enable_.no_go) {
    em::layers::addNoGoFromCost(map_.getRawGridMap(),
                                "multi_cost",
                                layers_.multi_cost_tau,
                                layers_.no_go_inflate_m,
                                "no_go");
  }
  if (enable_.frontier_filter) {
    em::layers::filterFrontierByNoGo(map_.getRawGridMap(),
                                    "frontier", "no_go",
                                    layers_.frontier_clearance_m,
                                    "frontier_ok");
  }

  // // (Opcional) Occupancy "like" para planners 2D
  // if (enable_.occupancy_like) {
  //   em::layers::addOccupancyLike(map_.getRawGridMap(),
  //                               "grid", "no_go", "occupancy_like");
  //   // Si tienes esta util, conviértelo a máscara 0/100/-1
  //   em::layers::occupancyLikeToMask(map_.getRawGridMap(),
  //                                   "occupancy_like", "occupancy_mask");
  // }

  // // Publicar /occupancy si hay subs
  // if (occupancyPub_ && occupancyPub_->get_subscription_count() > 0 &&
  //     map_.getRawGridMap().exists("occupancy_mask")) {
  //   nav_msgs::msg::OccupancyGrid og;
  //   grid_map::GridMapRosConverter::toOccupancyGrid(
  //       map_.getRawGridMap(), "occupancy_mask", 0.0, 1.0, og);  // ROS2: void
  //   og.header.stamp    = nodeHandle_->get_clock()->now();
  //   og.header.frame_id = map_.getFrameId();
  //   occupancyPub_->publish(og);
  // }


  // if (enable_.no_go) {
  //   if (layers_.no_go_use_multi_cost) {
  //     // MODO ANTIGUO (por si quieres dejarlo conmutado)
  //     em::layers::addNoGoFromCost(map_.getRawGridMap(),
  //                                 "multi_cost",
  //                                 layers_.multi_cost_tau,
  //                                 layers_.no_go_inflate_m,
  //                                 "no_go");
  //   } else {
  //     // NUEVO VETO DURO, SIN PENALIZAR CUESTAS TRAVESABLES
  //     em::layers::addNoGoHard(map_.getRawGridMap(),
  //                             layers_.no_go_slope_blocking_rad,
  //                             layers_.no_go_rough_blocking_m,
  //                             layers_.no_go_use_obstacles,
  //                             layers_.no_go_use_negatives,
  //                             layers_.no_go_use_cvar,
  //                             layers_.no_go_cvar_tau,
  //                             layers_.no_go_inflate_m,
  //                             "no_go");
  //   }
  // }


  if (enable_.obstacles_binary) {
    em::layers::addObstacleBinaryFromGeom(
        map_.getRawGridMap(),
        layers_.slope_long_max_rad,
        layers_.step_max_m);
  }

  // Capas pesadas cada N ticks
  if (tickCount_ % std::max(1, rates_.heavy_every_n) == 0) {
    if (enable_.negative) {
      em::layers::addNegativeObstaclesRobust(
          map_.getRawGridMap(),
          rates_.neg_drop_thresh_m,
          rates_.neg_ring_m,
          rates_.neg_min_valid_ratio,
          rates_.neg_slope_gate_rad,
          "elevation", "slope", "negatives");
    }
    if (enable_.cvar) {
      em::layers::addCvarTraversability(
          map_.getRawGridMap(),
          layers_.cvar_alpha,
          layers_.cvar_window_m /*, out="cvar_risk"*/);
    }
  }

// test for the noon go sites and marker how obstacles /////////////////////////////

{
auto& gm = map_.getRawGridMap();

  // Parámetros del robot (ajústalos a tu vehículo)
  const double mu        = 0.62;  // coef. fricción suelo-rueda
  const double h_cg      = 0.70;  // m, altura del centro de masas
  const double track     = 1.26;  // m, ancho de vía (rueda izq a der)
  const double step_max  = 400.15;  // m, escalón superable
  const double rough_max = 0.15;  // m RMS permisible
  const double safety    = 0.035;  // margen (rad) para ir conservador

  // Límites derivados (conservadores si no separas pitch/roll)
  const double theta_trac = std::atan(mu) - safety;                 // ascenso por tracción
  const double phi_roll   = std::atan(0.5 * track / h_cg) - safety; // vuelco lateral
  const double slope_lim  = std::min(theta_trac, phi_roll);

  // Capas requeridas
  if (!gm.exists("slope") || !gm.exists("rough") || !gm.exists("step") || !gm.exists("negatives")) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "no_go_cap: faltan capas base (slope/rough/step/negatives).");
  } else {
    const std::string out_cap = "no_go_cap";
    if (!gm.exists(out_cap)) gm.add(out_cap, 0.0);

    auto& CAP   = gm[out_cap];
    const auto& S   = gm["slope"];                        // rad
    const auto& R   = gm["rough"];                        // m
    const auto& ST  = gm["step"];                         // m
    const auto& NEG = gm["negatives"];                    // [0,1]
    const auto* RISKp = gm.exists("cvar_risk") ? &gm["cvar_risk"] : nullptr;

    // Usamos elevación (inpainted si existe) para distinguir conocido/desconocido
    const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
    const auto& E = gm[elev];

    for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);

      // Desconocido => deja NaN (para que navgrid ponga -1)
      const float e = E(idx(0), idx(1));
      if (std::isnan(e)) { 
        CAP(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN(); 
        continue; 
      }

      bool block = false;
      block |= (S (idx(0), idx(1)) > slope_lim);
      block |= (R (idx(0), idx(1)) > rough_max);
      block |= (ST(idx(0), idx(1)) > step_max);
      block |= (NEG(idx(0), idx(1)) > 0.5f);
      if (RISKp) block |= ((*RISKp)(idx(0), idx(1)) > 0.9f);

      CAP(idx(0), idx(1)) = block ? 1.0f : 0.0f;
    }
  }

}

//////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ---- NAVGRID desde NEGATIVES: desconocido/libre/ocupado (solo negatives) ----

  // auto& gm = map_.getRawGridMap();

  // // Usa elevation_inpainted si existe (evita marcar desconocido por pequeños huecos)
  // const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
  // const std::string neg  = "negatives";
  // const std::string out  = "navgrid";   // capa float: NaN (desconocido), 0.0 (libre), 1.0 (ocupado)

  // if (!gm.exists(elev) || !gm.exists(neg)) {
  //   RCLCPP_WARN(nodeHandle_->get_logger(),
  //               "navgrid: faltan capas '%s' o '%s'", elev.c_str(), neg.c_str());
  // } else {
  //   if (!gm.exists(out)) gm.add(out, std::numeric_limits<float>::quiet_NaN());

  //   grid_map::Matrix& L       = gm[out];
  //   const grid_map::Matrix& E = gm[elev];
  //   const grid_map::Matrix& N = gm[neg];

  //   const float tau = 0.50f;  // negatives > tau => OCUPADO. Ajusta a tu gusto.

  //   for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
  //     const grid_map::Index idx(*it);
  //     const float e = E(idx(0), idx(1));
  //     if (std::isnan(e)) {
  //       // no observado => desconocido
  //       L(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
  //     } else {
  //       const float nv = N(idx(0), idx(1));
  //       L(idx(0), idx(1)) = (nv > tau) ? 1.0f : 0.0f;  // 1=ocupado, 0=libre
  //     }
  //   }

  //   // Publica OccupancyGrid desde 'navgrid': 0->0, 1->100, NaN->-1
  //   if (occupancyPub_ && occupancyPub_->get_subscription_count() > 0) {
  //     nav_msgs::msg::OccupancyGrid og;
  //     grid_map::GridMapRosConverter::toOccupancyGrid(gm, out, 0.0f, 1.0f, og);
  //     og.header.stamp    = nodeHandle_->get_clock()->now();
  //     og.header.frame_id = gm.getFrameId();
  //     occupancyPub_->publish(og);
  //   }
  // }
////////////////////////////////////////////////////////////////////////////////////// 

//////////////////// 
//navgird con constantes
{
// auto& gm = map_.getRawGridMap();
//   const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
//   const std::string cap  = "no_go_cap";
//   const std::string out  = "navgrid"; // NaN=desconocido, 0.0=libre, 1.0=ocupado

//   if (!gm.exists(elev) || !gm.exists(cap)) {
//     RCLCPP_WARN(nodeHandle_->get_logger(), "navgrid: faltan capas '%s' o '%s'", elev.c_str(), cap.c_str());
//   } else {
//     if (!gm.exists(out)) gm.add(out, std::numeric_limits<float>::quiet_NaN());

//     auto& L       = gm[out];
//     const auto& E = gm[elev];
//     const auto& C = gm[cap];

//     for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
//       const grid_map::Index idx(*it);
//       const float e = E(idx(0), idx(1));
//       if (std::isnan(e)) {
//         L(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
//       } else {
//         L(idx(0), idx(1)) = (C(idx(0), idx(1)) > 0.5f) ? 1.0f : 0.0f;
//       }
//     }

//     // Publica OccupancyGrid (0->0, 1->100, NaN->-1)
//     if (occupancyPub_ && occupancyPub_->get_subscription_count() > 0) {
//       nav_msgs::msg::OccupancyGrid og;
//       grid_map::GridMapRosConverter::toOccupancyGrid(gm, out, 0.0f, 1.0f, og);
//       og.header.stamp    = nodeHandle_->get_clock()->now();
//       og.header.frame_id = gm.getFrameId();
//       occupancyPub_->publish(og);
//     }
//   }
// --- NAVGRID desde NEGATIVES con tau parametrizable + filtro anti-puntos ---
auto& gm = map_.getRawGridMap();
const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
const std::string neg  = "negatives";
const std::string out  = "navgrid";

double navgrid_tau = 0.65;
nodeHandle_->get_parameter("navgrid_tau", navgrid_tau);

if (gm.exists(elev) && gm.exists(neg)) {
  if (!gm.exists(out)) gm.add(out, std::numeric_limits<float>::quiet_NaN());

  grid_map::Matrix& L       = gm[out];
  const grid_map::Matrix& E = gm[elev];
  const grid_map::Matrix& N = gm[neg];

  // 1) Umbral
  for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
    const grid_map::Index idx(*it);
    const float e = E(idx(0), idx(1));
    if (std::isnan(e)) {
      L(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();  // desconocido
    } else {
      const float nv = N(idx(0), idx(1));
      L(idx(0), idx(1)) = (nv > static_cast<float>(navgrid_tau)) ? 1.0f : 0.0f;  // 1=ocupado, 0=libre
    }
  }

  // 2) Filtro 3×3 para eliminar puntos sueltos ocupados (apertura suave)
  grid_map::Matrix Lcopy = L;
  auto is_occ = [&](int x, int y)->bool {
    const float v = Lcopy(x,y);
    return (!std::isnan(v) && v > 0.5f);
  };

  const auto size = gm.getSize(); // size(0)=Nx, size(1)=Ny


  auto inBounds = [&](int x, int y) -> bool {
  return (x >= 0 && y >= 0 && x < size(0) && y < size(1));
};

for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
  const grid_map::Index idx(*it);
  const float v = Lcopy(idx(0), idx(1));

  if (!std::isfinite(v)) continue;  // dejamos desconocidos como están
  if (v <= 0.5f) continue;          // solo tocamos ocupados

  int occ_nb = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;

      const int nx = idx(0) + dx;
      const int ny = idx(1) + dy;
      if (!inBounds(nx, ny)) continue;       // <-- reemplaza la llamada errónea
      if (is_occ(nx, ny)) ++occ_nb;
    }
  }

  if (occ_nb <= 1) {
    // si está solo o casi solo, bórralo
    L(idx(0), idx(1)) = 0.0f;
  }
}

  // Publicar OccupancyGrid (0->libre, 1->ocupado, NaN->desconocido)
  if (occupancyPub_ && occupancyPub_->get_subscription_count() > 0) {
    nav_msgs::msg::OccupancyGrid og;
    grid_map::GridMapRosConverter::toOccupancyGrid(gm, out, 0.0f, 1.0f, og);
    og.header.stamp    = nodeHandle_->get_clock()->now();
    og.header.frame_id = gm.getFrameId();
    occupancyPub_->publish(og);
  }
} else {
  RCLCPP_WARN(nodeHandle_->get_logger(), "navgrid: faltan capas '%s' o '%s'", elev.c_str(), neg.c_str());
}
}
////////////////////////////////

{

  // ---- DEBUG: capa 'cap_debug' (motivo de bloqueo) ----
auto& gm = map_.getRawGridMap();

const std::string elev = gm.exists("elevation_inpainted") ? "elevation_inpainted" : "elevation";
const bool hasVar   = gm.exists("variance");
const bool hasNeg   = gm.exists("negatives");
const bool hasCVar  = gm.exists("cvar_risk");
const bool hasSlope = gm.exists("slope");
const bool hasRough = gm.exists("rough");
const bool hasStep  = gm.exists("step");

const float var_tau   = static_cast<float>(layers_.grid_var_thresh);        // p.ej. 0.02–0.03 m^2
const float neg_tau   = 0.50f;                                              // umbral para 'negatives'
const float cvar_tau  = static_cast<float>(layers_.no_go_cvar_tau);         // umbral CVaR
const float slope_blk = static_cast<float>(layers_.no_go_slope_blocking_rad);
const float rough_blk = static_cast<float>(layers_.no_go_rough_blocking_m);
const float step_blk  = static_cast<float>(layers_.step_max_m);

const std::string out_dbg = "cap_debug";
if (!gm.exists(out_dbg)) gm.add(out_dbg, std::numeric_limits<float>::quiet_NaN());
grid_map::Matrix& D = gm[out_dbg];

std::array<int, 6> hist = {0,0,0,0,0,0};  // 0..5

for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
  const grid_map::Index idx(*it);

  // 0) desconocido (sin elevación) => VAR
  const float e = gm[elev](idx(0), idx(1));
  int reason = 0;
  if (std::isnan(e)) {
    reason = 5;
  }
  // 1) varianza alta => VAR
  else if (hasVar) {
    const float v = gm["variance"](idx(0), idx(1));
    if (std::isfinite(v) && v > var_tau) {
      reason = 5;
    }
  }

  // 2) riesgos (negativos/CVaR) => RISK
  if (reason == 0) {
    bool risky = false;
    if (hasNeg)  risky = risky || (gm["negatives"](idx(0), idx(1)) > neg_tau);
    if (hasCVar) risky = risky || (gm["cvar_risk"](idx(0), idx(1))  > cvar_tau);
    if (risky) reason = 4;
  }

  // 3) geometría dura: STEP > umbral
  if (reason == 0 && hasStep) {
    if (gm["step"](idx(0), idx(1)) > step_blk) reason = 3;
  }

  // 4) ROUGH > umbral
  if (reason == 0 && hasRough) {
    if (gm["rough"](idx(0), idx(1)) > rough_blk) reason = 2;
  }

  // 5) SLOPE > umbral
  if (reason == 0 && hasSlope) {
    if (gm["slope"](idx(0), idx(1)) > slope_blk) reason = 1;
  }

  // 6) OK
  D(idx(0), idx(1)) = static_cast<float>(reason);
  if (reason >= 0 && reason < static_cast<int>(hist.size())) hist[reason]++;
}

// Log resumen (útil para ver rápido qué está bloqueando más)
const int tot = std::accumulate(hist.begin(), hist.end(), 0);
RCLCPP_INFO(nodeHandle_->get_logger(),
  "cap_debug: OK=%d SLOPE=%d ROUGH=%d STEP=%d RISK=%d VAR=%d (tot=%d)",
  hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], tot);
}
{
  auto& gm = map_.getRawGridMap();
  if (!gm.exists("cap_debug")) { /* nada que pintar */ }
  else {
    const double res = gm.getResolution();

    // Prepara 6 marcadores (uno por motivo).
    std::array<visualization_msgs::msg::Marker, 6> ms;
    const std::array<std::array<float,4>,6> colors = {{
      {0.0f, 1.0f, 0.0f, 0.6f},  // 0 OK      -> verde
      {1.0f, 0.5f, 0.0f, 0.8f},  // 1 SLOPE   -> naranja
      {1.0f, 1.0f, 0.0f, 0.8f},  // 2 ROUGH   -> amarillo
      {1.0f, 0.0f, 0.0f, 0.8f},  // 3 STEP    -> rojo
      {1.0f, 0.0f, 1.0f, 0.8f},  // 4 RISK    -> magenta
      {0.5f, 0.5f, 0.5f, 0.8f}   // 5 VAR     -> gris
    }};
    const auto stamp = nodeHandle_->get_clock()->now();
    for (int r = 0; r < 6; ++r) {
      auto& m = ms[r];
      m.header.frame_id = gm.getFrameId();
      m.header.stamp    = stamp;
      m.ns   = "cap_debug";
      m.id   = r;
      m.type = visualization_msgs::msg::Marker::CUBE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = res; m.scale.y = res; m.scale.z = 0.01;
      m.color.r = colors[r][0]; m.color.g = colors[r][1];
      m.color.b = colors[r][2]; m.color.a = colors[r][3];
      m.pose.orientation.w = 1.0;  // identidad
    }

    // Mete cada celda en su marcador por categoría.
    const grid_map::Matrix& D = gm["cap_debug"];
    for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
      grid_map::Index idx(*it);
      float v = D(idx(0), idx(1));
      if (!std::isfinite(v)) continue;
      int reason = static_cast<int>(std::round(v));
      if (reason < 0 || reason > 5) continue;

      grid_map::Position p; gm.getPosition(*it, p);
      geometry_msgs::msg::Point q;
      q.x = p.x(); q.y = p.y(); q.z = 0.02;  // un pelín alto para que se vea
      ms[reason].points.push_back(q);
    }

    visualization_msgs::msg::MarkerArray arr;
    for (auto& m : ms) arr.markers.push_back(m);
    debugMarkersPub_->publish(arr);
  }
}



  // // Capa final
  // em::layers::combineToFinalObstacles(
  //     map_.getRawGridMap(),
  //     "obstacles", "negatives", "cvar_risk",
  //     rates_.cvar_tau, "obstacles_final");

  // Publicación única
  map_.postprocessAndPublishRawElevationMap();
  if (isFusingEnabled()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }
}

void ElevationMapping::publishFusedMapCallback() {
  if (!map_.hasFusedMapSubscribers()) {
    return;
  }
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback() {
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_.visibilityCleanup(rclcpp::Time(lastPointCloudUpdateTime_, RCL_ROS_TIME));
}

bool ElevationMapping::fuseEntireMapServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                                    const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
  return true;
}

bool ElevationMapping::isFusingEnabled() {
  return isContinuouslyFusing_ && map_.hasFusedMapSubscribers();
}

bool ElevationMapping::updatePrediction(const rclcpp::Time& time) {
  if (ignoreRobotMotionUpdates_) {
    return true;
  }

  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().seconds());

  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Requested update with time stamp %f, but time of last update was %f.",
                 time.seconds(), map_.getTimeOfLastUpdate().seconds());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Requested update with time stamp %f, but time of last update was %f. Ignoring update.",
                 time.seconds(), map_.getTimeOfLastUpdate().seconds());
    return true;
  }

  // Get robot pose at requested time.
  std::shared_ptr<const nav_msgs::msg::Odometry> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
    if (robotPoseCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds()) {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "The oldest pose available is at %f, requested pose at %f",
                   robotPoseCache_.getOldestTime().seconds(), lastPointCloudUpdateTime_.seconds());
    } else {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?",
                   lastPointCloudUpdateTime_.seconds());
    }
    return false;
  }

  kindr::HomTransformQuatD robotPose;
  kindr_ros::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation() {
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map is checked for relocalization.");

  geometry_msgs::msg::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = rclcpp::Time(0);
  kindr_ros::convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::msg::PointStamped trackPointTransformed;

  try {
    trackPointTransformed = transformBuffer_->transform(trackPoint, map_.getFrameId());
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "%s", ex.what());
    return false;
  }

  kindr::Position3D position3d;
  kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_.move(position);
  return true;
}

bool ElevationMapping::getFusedSubmapServiceCallback(std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                     std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.",
               requestedSubmapPosition.x(), requestedSubmapPosition.y(),
               requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess;
  grid_map::GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  scopedLock.unlock();

  if (request->layers.empty()) {
    response->map = *grid_map::GridMapRosConverter::toMessage(subMap);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    response->map = *grid_map::GridMapRosConverter::toMessage(subMap, layers);
  }

  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().seconds());
  return isSuccess;
}

bool ElevationMapping::getRawSubmapServiceCallback(std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                   std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.",
               requestedSubmapPosition.x(), requestedSubmapPosition.y(),
               requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  bool isSuccess;
  grid_map::GridMap subMap = map_.getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  scopedLock.unlock();

  if (request->layers.empty()) {
    response->map = *grid_map::GridMapRosConverter::toMessage(subMap);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    response->map = *grid_map::GridMapRosConverter::toMessage(subMap, layers);
  }
  return isSuccess;
}

bool ElevationMapping::disableUpdatesServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                                     const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                     std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Disabling updates.");
  updatesEnabled_ = false;
  return true;
}

bool ElevationMapping::enableUpdatesServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                                    const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                                    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Enabling updates.");
  updatesEnabled_ = true;
  return true;
}

bool ElevationMapping::initializeElevationMap() {
  if (initializeElevationMap_) {
    if (static_cast<elevation_mapping::InitializationMethods>(initializationMethod_) ==
        elevation_mapping::InitializationMethods::PlanarFloorInitializer) {
      geometry_msgs::msg::TransformStamped transform_msg;
      tf2::Stamped<tf2::Transform> transform;

      // Listen to transform between mapFrameId_ and targetFrameInitSubmap_ and use z value for initialization
      try {
        transform_msg = transformBuffer_->lookupTransform(mapFrameId_, targetFrameInitSubmap_, rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0));
        tf2::fromMsg(transform_msg, transform);

        RCLCPP_DEBUG_STREAM(nodeHandle_->get_logger(), "Initializing with x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y()
                                                 << " z: " << transform.getOrigin().z());

        const grid_map::Position positionRobot(transform.getOrigin().x(), transform.getOrigin().y());

        // Move map before we apply the height values. This prevents unwanted behavior from intermediate move() calls in
        // updateMapLocation().
        map_.move(positionRobot);

        map_.setRawSubmapHeight(positionRobot, transform.getOrigin().z() + initSubmapHeightOffset_, lengthInXInitSubmap_,
                                lengthInYInitSubmap_, marginInitSubmap_);
        return true;
      } catch (tf2::TransformException& ex) {
        RCLCPP_DEBUG(nodeHandle_->get_logger(), "%s", ex.what());
        RCLCPP_WARN(nodeHandle_->get_logger(), "Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
        return false;
      }
    }
  }
  return true;
}

bool ElevationMapping::clearMapServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                               const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                               std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Clearing map...");
  bool success = map_.clear();
  success &= initializeElevationMap();
  RCLCPP_INFO(nodeHandle_->get_logger(), "Map cleared.");

  return success;
}

bool ElevationMapping::maskedReplaceServiceCallback(std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request,
                                                    std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> /*response*/) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request->map, sourceMap);

  // Use the supplied mask or do not use a mask
  grid_map::Matrix mask;
  if (sourceMap.exists(maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  boost::recursive_mutex::scoped_lock scopedLockRawData(map_.getRawDataMutex());

  // Loop over all layers that should be set
  for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
       sourceLayerIterator++) {
    // skip "mask" layer
    if (*sourceLayerIterator == maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[*sourceLayerIterator];
    // Check if the layer exists in the elevation map
    if (map_.getRawGridMap().exists(*sourceLayerIterator)) {
      grid_map::Matrix& destinationLayer = map_.getRawGridMap()[*sourceLayerIterator];
      for (grid_map::GridMapIterator destinationIterator(map_.getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator) {
        // Use the position to find corresponding indices in source and destination
        const grid_map::Index destinationIndex(*destinationIterator);
        grid_map::Position position;
        map_.getRawGridMap().getPosition(*destinationIterator, position);

        if (!sourceMap.isInside(position)) {
          continue;
        }

        grid_map::Index sourceIndex;
        sourceMap.getIndex(position, sourceIndex);
        // If the mask allows it, set the value from source to destination
        if (!std::isnan(mask(sourceIndex(0), sourceIndex(1)))) {
          destinationLayer(destinationIndex(0), destinationIndex(1)) = sourceLayer(sourceIndex(0), sourceIndex(1));
        }
      }
    } else {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
    }
  }

  return true;
}

bool ElevationMapping::saveMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Saving map to file.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  std::string topic = std::string(nodeHandle_->get_namespace()) + "/elevation_map";
  if (!request->topic_name.empty()) {
    topic = std::string(nodeHandle_->get_namespace()) + "/" + request->topic_name;
  }
  response->success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(map_.getFusedGridMap(), request->file_path, topic));
  response->success = static_cast<unsigned char>(
      (grid_map::GridMapRosConverter::saveToBag(map_.getRawGridMap(), request->file_path + "_raw", topic + "_raw")) &&
      static_cast<bool>(response->success));
  return static_cast<bool>(response->success);
}

bool ElevationMapping::loadMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_WARN(nodeHandle_->get_logger(), "Loading from bag file.");
  boost::recursive_mutex::scoped_lock scopedLockFused(map_.getFusedDataMutex());
  boost::recursive_mutex::scoped_lock scopedLockRaw(map_.getRawDataMutex());

  std::string topic = nodeHandle_->get_namespace();
  if (!request->topic_name.empty()) {
    topic += "/" + request->topic_name;
  } else {
    topic += "/elevation_map";
  }

  response->success =
      static_cast<unsigned char>(grid_map::GridMapRosConverter::loadFromBag(request->file_path, topic, map_.getFusedGridMap()));
  response->success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::loadFromBag(request->file_path + "_raw", topic + "_raw", map_.getRawGridMap()) &&
      static_cast<bool>(response->success));

  // Update timestamp for visualization in ROS
  map_.setTimestamp(nodeHandle_->get_clock()->now());
  map_.postprocessAndPublishRawElevationMap();
  return static_cast<bool>(response->success);
}

// void ElevationMapping::resetMapUpdateTimer() {
//   mapUpdateTimer_->cancel();
//   rclcpp::Duration periodSinceLastUpdate = nodeHandle_->get_clock()->now() - map_.getTimeOfLastUpdate();
//   if (periodSinceLastUpdate > maxNoUpdateDuration_) {
//     periodSinceLastUpdate = rclcpp::Duration::from_seconds(0.0);
//   }
//   mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
//   mapUpdateTimer_.start();
// }

// void ElevationMapping::stopMapUpdateTimer() {
//   mapUpdateTimer_->cancel();
// }

}  // namespace elevation_mapping
