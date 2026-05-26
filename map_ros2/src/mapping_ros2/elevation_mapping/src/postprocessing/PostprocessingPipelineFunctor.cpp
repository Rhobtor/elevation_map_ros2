/*
 * PostprocessingPipelineFunctor.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *  Note. Large parts are adopted from grid_map_demos/FiltersDemo.cpp.
 */

#include <grid_map_ros/grid_map_ros.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace elevation_mapping {

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(std::shared_ptr<rclcpp::Node> nodeHandle)
    : nodeHandle_(nodeHandle), filterChain_("grid_map::GridMap"), filterChainConfigured_(false) {
  // TODO (magnus) Add logic when setting up failed. What happens actually if it is not configured?
  if (!nodeHandle->has_parameter("filterChainParametersName_")){
    nodeHandle_->declare_parameter("filterChainParametersName_", std::string("postprocessor_pipeline"));
  }
 
  readParameters();

  publisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>(outputTopic_, 1);
  if (!secondaryOutputTopic_.empty() && secondaryOutputTopic_ != outputTopic_) {
    secondaryPublisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>(secondaryOutputTopic_, 1);
  }

  // Setup filter chain.  
  if (!nodeHandle->has_parameter("filterChainParametersName_") ||
      !filterChain_.configure(filterChainParametersName_, nodeHandle_->get_node_logging_interface(), nodeHandle_->get_node_parameters_interface())) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Could not configure the filter chain. Will publish the raw elevation map without postprocessing!");
    return;
  }
  
  filterChainConfigured_ = true;
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() { 
  if (!nodeHandle_->has_parameter("output_layers")) {
    nodeHandle_->declare_parameter("output_layers", std::vector<std::string>{});
  }
  if (!nodeHandle_->has_parameter("secondary_output_topic")) {
    nodeHandle_->declare_parameter("secondary_output_topic", std::string());
  }
  nodeHandle_->get_parameter("output_topic", outputTopic_);
  nodeHandle_->get_parameter("secondary_output_topic", secondaryOutputTopic_);
  nodeHandle_->get_parameter("output_layers", outputLayers_);
  nodeHandle_->get_parameter("postprocessor_pipeline_name", filterChainParametersName_);
  
  }

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap& inputMap) {
  if (not filterChainConfigured_) {    
    RCLCPP_WARN_ONCE(nodeHandle_->get_logger(), "No postprocessing pipeline was configured. Forwarding the raw elevation map!");
    return inputMap;
  }
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "performing Post processing");
  grid_map::GridMap outputMap;
  if (not filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not perform the grid map filter chain! Forwarding the raw elevation map!");
    return inputMap;
  }

  return outputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap& gridMap) const {
  // Publish filtered output grid map.
  std::vector<std::string> availableLayers;
  if (!outputLayers_.empty()) {
    availableLayers.reserve(outputLayers_.size());
    for (const auto& layer : outputLayers_) {
      if (gridMap.exists(layer)) {
        availableLayers.push_back(layer);
      }
    }
  }
  std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
  if (availableLayers.empty()) {
    outputMessage = outputLayers_.empty()
                        ? grid_map::GridMapRosConverter::toMessage(gridMap)
                        : grid_map::GridMapRosConverter::toMessage(gridMap, availableLayers);
  } else {
    outputMessage = grid_map::GridMapRosConverter::toMessage(gridMap, availableLayers);
  }
  publisher_->publish(std::move(outputMessage));  
  if (secondaryPublisher_) {
    std::unique_ptr<grid_map_msgs::msg::GridMap> secondaryMessage;
    if (availableLayers.empty()) {
      secondaryMessage = outputLayers_.empty()
                             ? grid_map::GridMapRosConverter::toMessage(gridMap)
                             : grid_map::GridMapRosConverter::toMessage(gridMap, availableLayers);
    } else {
      secondaryMessage = grid_map::GridMapRosConverter::toMessage(gridMap, availableLayers);
    }
    secondaryPublisher_->publish(std::move(secondaryMessage));
  }
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Elevation map raw has been published.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_->get_subscription_count() > 0 ||
         (secondaryPublisher_ && secondaryPublisher_->get_subscription_count() > 0);
}

}  // namespace elevation_mapping
