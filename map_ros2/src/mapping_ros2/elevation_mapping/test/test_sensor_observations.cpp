#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <memory>
#include <vector>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include "elevation_mapping/ElevationMap.hpp"

namespace
{

class SensorObservations : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::unique_ptr<elevation_mapping::ElevationMap> makeMap(const std::string& name)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"use_sim_time", true}, {"postprocessor_pipeline_name", ""}, {"fused_output_topic", ""}});
    auto node = std::make_shared<rclcpp::Node>(name, options);
    node->declare_parameter("postprocessor_num_threads", 1);
    auto map = std::make_unique<elevation_mapping::ElevationMap>(node);
    map->setFrameId("map");
    map->setGeometry(grid_map::Length(6, 6), 0.5, grid_map::Position(4, -2));
    return map;
  }

  elevation_mapping::PointCloudType::Ptr cloud(
    const std::vector<Eigen::Vector3f>& positions)
  {
    auto result = std::make_shared<elevation_mapping::PointCloudType>();
    for (const auto& position : positions) {
      elevation_mapping::PointCloudType::PointType point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.r = point.g = point.b = 0;
      result->push_back(point);
    }
    return result;
  }

  bool add(elevation_mapping::ElevationMap& map, const std::vector<Eigen::Vector3f>& points,
    const std::int64_t time, grid_map::GridMap* observations)
  {
    auto input = cloud(points);
    Eigen::VectorXf variances = Eigen::VectorXf::Constant(points.size(), 0.0001F);
    return map.add(input, variances, rclcpp::Time(time, RCL_ROS_TIME),
      Eigen::Affine3d::Identity(), observations);
  }
};

TEST_F(SensorObservations, MaximumIsConservativeAndVarianceBelongsToThatMeasurement)
{
  auto map = makeMap("aggregation_test");
  auto input = cloud({{4.25F, -1.75F, 0}, {4.25F, -1.75F, 2}, {4.25F, -1.75F, 1}});
  Eigen::VectorXf variances(3);
  variances << 0.01F, 0.04F, 0.02F;
  grid_map::GridMap observations;
  ASSERT_TRUE(map->add(input, variances, rclcpp::Time(1000000001LL, RCL_ROS_TIME),
    Eigen::Affine3d::Identity(), &observations));
  grid_map::Index index;
  ASSERT_TRUE(observations.getIndex(grid_map::Position(4.25, -1.75), index));
  EXPECT_FLOAT_EQ(observations.at("elevation", index), 2);
  EXPECT_FLOAT_EQ(observations.at("minimum_elevation", index), 0);
  EXPECT_FLOAT_EQ(observations.at("measurement_variance", index), 0.04F);
  EXPECT_FLOAT_EQ(observations.at("point_count", index), 3);
  EXPECT_FLOAT_EQ(observations.at("observation_provenance", index), 1);
  EXPECT_EQ(observations.getTimestamp(), 1000000001ULL);
  EXPECT_EQ(observations.getFrameId(), "map");
  EXPECT_EQ(observations["elevation"].array().isFinite().count(), 1);
}

TEST_F(SensorObservations, FreshLowerMeasurementsDoNotCopyTheStoredObstacle)
{
  auto map = makeMap("lower_test");
  grid_map::GridMap observations;
  ASSERT_TRUE(add(*map, {{4.25F, -1.75F, 2}}, 1000000000LL, &observations));
  ASSERT_TRUE(add(*map, {{4.25F, -1.75F, 0}}, 2100000000LL, &observations));
  grid_map::Index index;
  ASSERT_TRUE(observations.getIndex(grid_map::Position(4.25, -1.75), index));
  EXPECT_FLOAT_EQ(observations.at("elevation", index), 0);
  EXPECT_FLOAT_EQ(map->getRawGridMap().at("elevation", index), 2);
}

TEST_F(SensorObservations, AsymmetricWorldPositionsAndCircularBufferPreserveUnknownCells)
{
  auto map = makeMap("geometry_test");
  map->move(grid_map::Position(4.5, -2.5));
  ASSERT_GT(map->getRawGridMap().getStartIndex().matrix().squaredNorm(), 0);
  grid_map::GridMap observations;
  ASSERT_TRUE(add(*map, {{2.25F, -4.75F, 1.7F}, {6.75F, -0.25F, -0.3F}},
    1000000000LL, &observations));
  for (const Eigen::Vector3f& point : {
      Eigen::Vector3f(2.25F, -4.75F, 1.7F), Eigen::Vector3f(6.75F, -0.25F, -0.3F)}) {
    grid_map::Index index;
    ASSERT_TRUE(observations.getIndex(point.head<2>().cast<double>(), index));
    EXPECT_FLOAT_EQ(observations.at("elevation", index), point.z());
    grid_map::Position world;
    ASSERT_TRUE(observations.getPosition(index, world));
    EXPECT_DOUBLE_EQ(world.x(), point.x());
    EXPECT_DOUBLE_EQ(world.y(), point.y());
  }
  EXPECT_EQ(observations["elevation"].array().isFinite().count(), 2);
  ASSERT_TRUE(add(*map, {}, 2000000000LL, &observations));
  EXPECT_EQ(observations["elevation"].array().isFinite().count(), 0);
  EXPECT_EQ(observations["observation_provenance"].sum(), 0);
}

TEST_F(SensorObservations, EveryLegacyLayerIsIdenticalWithAndWithoutExport)
{
  auto baseline = makeMap("baseline_test");
  auto enabled = makeMap("enabled_test");
  grid_map::GridMap observations;
  for (int scan = 1; scan <= 8; ++scan) {
    if (scan == 4) {
      baseline->move(grid_map::Position(4.5, -2.5));
      enabled->move(grid_map::Position(4.5, -2.5));
    }
    const std::vector<Eigen::Vector3f> points{
      {4.25F, -1.75F, scan < 4 ? 1.0F : 0.0F},
      {4.25F, -1.75F, scan == 2 ? 2.0F : 0.0F},
      {6.25F, -3.75F, 0.3F}};
    const auto stamp = static_cast<std::int64_t>(scan) * 1100000000LL;
    ASSERT_TRUE(add(*baseline, points, stamp, nullptr));
    ASSERT_TRUE(add(*enabled, points, stamp, &observations));
    const auto& a = baseline->getRawGridMap();
    const auto& b = enabled->getRawGridMap();
    ASSERT_EQ(a.getLayers(), b.getLayers());
    EXPECT_EQ(a.getTimestamp(), b.getTimestamp());
    EXPECT_TRUE((a.getStartIndex() == b.getStartIndex()).all());
    for (const auto& layer : a.getLayers()) {
      const auto& first = a[layer];
      const auto& second = b[layer];
      for (Eigen::Index i = 0; i < first.size(); ++i) {
        if (std::isnan(first.data()[i])) {
          EXPECT_TRUE(std::isnan(second.data()[i])) << layer;
        } else {
          EXPECT_EQ(first.data()[i], second.data()[i]) << layer;
        }
      }
    }
  }
}

TEST_F(SensorObservations, MeasuresPairedCpuAndLatencyOverhead)
{
  auto baseline = makeMap("baseline_performance");
  auto enabled = makeMap("enabled_performance");
  baseline->setGeometry(grid_map::Length(21, 21), 0.15, grid_map::Position::Zero());
  enabled->setGeometry(grid_map::Length(21, 21), 0.15, grid_map::Position::Zero());
  std::vector<Eigen::Vector3f> positions;
  for (int r = 0; r < 140; ++r) {
    for (int c = 0; c < 140; ++c) {
      positions.emplace_back(-10.5F + (r + 0.5F) * 0.15F,
        -10.5F + (c + 0.5F) * 0.15F, c == 75 ? 0.4F : 0);
    }
  }
  auto input = cloud(positions);
  Eigen::VectorXf variances = Eigen::VectorXf::Constant(positions.size(), 0.0001F);
  grid_map::GridMap observations;
  std::vector<double> wall[2];
  double cpu[2]{0, 0};
  for (int scan = 0; scan < 90; ++scan) {
    for (int order = 0; order < 2; ++order) {
      const int mode = (scan + order) % 2;
      const auto begin = std::chrono::steady_clock::now();
      const auto cpu_begin = std::clock();
      ASSERT_TRUE((mode == 0 ? baseline : enabled)->add(input, variances,
        rclcpp::Time(1000000000LL + scan * 100000000LL, RCL_ROS_TIME),
        Eigen::Affine3d::Identity(), mode == 0 ? nullptr : &observations));
      if (mode != 0) {
        const auto message = grid_map::GridMapRosConverter::toMessage(observations);
        ASSERT_EQ(message->layers.size(), 5U);
      }
      const auto cpu_end = std::clock();
      if (scan >= 10) {
        wall[mode].push_back(std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - begin).count());
        cpu[mode] += 1000.0 * (cpu_end - cpu_begin) / CLOCKS_PER_SEC;
      }
    }
  }
  for (int mode = 0; mode < 2; ++mode) {
    std::sort(wall[mode].begin(), wall[mode].end());
    EXPECT_GT(wall[mode].front(), 0);
    std::cout << "EXPORT_BENCHMARK mode=" << (mode ? "enabled" : "disabled")
      << " points=19600 cells=19600 samples=80 includes_export_serialization=1 median_ms=" << wall[mode][40]
      << " p95_ms=" << wall[mode][76] << " mean_cpu_ms=" << cpu[mode] / 80 << '\n';
  }
}

}  // namespace
