#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace obstacle_layers {

static inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static inline float sigmoid(float x) {
  // Safe-ish sigmoid.
  if (x >= 20.0f) return 1.0f;
  if (x <= -20.0f) return 0.0f;
  return 1.0f / (1.0f + std::exp(-x));
}

static inline bool finite(float v) {
  return std::isfinite(v);
}

static float quantile_inplace(std::vector<float>& values, float q) {
  if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
  q = std::clamp(q, 0.0f, 1.0f);
  const size_t k = static_cast<size_t>(std::floor(q * static_cast<float>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + static_cast<long>(k), values.end());
  return values[k];
}

struct CellReservoir {
  static constexpr size_t kMaxSamples = 32;

  std::array<float, kMaxSamples> samples{};
  uint8_t count{0};
  uint32_t seen{0};
  uint32_t total_points{0};
  uint32_t band_points{0};

  void reset() {
    count = 0;
    seen = 0;
    total_points = 0;
    band_points = 0;
  }

  template <class URNG>
  void observeSample(float v, URNG& rng) {
    ++seen;
    if (count < kMaxSamples) {
      samples[count++] = v;
      return;
    }
    std::uniform_int_distribution<uint32_t> dist(0, seen - 1);
    const uint32_t j = dist(rng);
    if (j < kMaxSamples) {
      samples[j] = v;
    }
  }

  float q90() const {
    if (count == 0) return 0.0f;
    std::array<float, kMaxSamples> tmp = samples;
    const size_t n = static_cast<size_t>(count);
    const size_t k = static_cast<size_t>(std::floor(0.90f * static_cast<float>(n - 1)));
    std::nth_element(tmp.begin(), tmp.begin() + static_cast<long>(k), tmp.begin() + static_cast<long>(n));
    return tmp[k];
  }

  float verticalOcc() const {
    if (total_points == 0) return 0.0f;
    return static_cast<float>(band_points) / (static_cast<float>(total_points) + 1e-6f);
  }
};

class ObstacleLayersNode final : public rclcpp::Node {
 public:
  ObstacleLayersNode() : Node("obstacle_layers") {
    input_map_topic_ = declare_parameter<std::string>("input_map_topic", "/elevation_mapping/elevation_map_raw");
    output_map_topic_ = declare_parameter<std::string>("output_map_topic", "/elevation_mapping/elevation_map_obstacles");
    input_cloud_topic_ = declare_parameter<std::string>("input_pointcloud_topic", std::string());

    elevation_layer_ = declare_parameter<std::string>("elevation_layer", "elevation");
    variance_layer_ = declare_parameter<std::string>("variance_layer", "variance");
    slope_layer_ = declare_parameter<std::string>("slope_layer", "slope");
    step_layer_ = declare_parameter<std::string>("step_layer", "step");
    rough_layer_ = declare_parameter<std::string>("rough_layer", "rough");
    clearance_layer_ = declare_parameter<std::string>("clearance_layer", "clearance");
    negatives_layer_ = declare_parameter<std::string>("negatives_layer", "negatives");

    support_window_m_ = declare_parameter<double>("support_window_m", 0.45);
    support_percentile_ = static_cast<float>(declare_parameter<double>("support_percentile", 0.20));
    variance_min_ = static_cast<float>(declare_parameter<double>("variance_min", 0.01));
    variance_max_ = static_cast<float>(declare_parameter<double>("variance_max", 0.05));

    step_hard_ = static_cast<float>(declare_parameter<double>("step_hard", 0.25));
    rough_hard_ = static_cast<float>(declare_parameter<double>("rough_hard", 0.10));
    slope_hard_ = static_cast<float>(declare_parameter<double>("slope_hard", 0.52));
    neg_hard_ = static_cast<float>(declare_parameter<double>("neg_hard", 0.25));
    h_soft_ = static_cast<float>(declare_parameter<double>("h_soft", 0.10));
    h_hard_ = static_cast<float>(declare_parameter<double>("h_hard", 0.25));
    clear_min_ = static_cast<float>(declare_parameter<double>("clear_min", 0.40));
    h_climb_ = static_cast<float>(declare_parameter<double>("h_climb", 0.10));
    h_body_ = static_cast<float>(declare_parameter<double>("h_body", 0.45));

    alpha_ = static_cast<float>(declare_parameter<double>("alpha", 0.85));

    kh_ = static_cast<float>(declare_parameter<double>("kh", 8.0));
    ks_ = static_cast<float>(declare_parameter<double>("ks", 6.0));
    Th_ = static_cast<float>(declare_parameter<double>("Th", 0.55));
    Ts_ = static_cast<float>(declare_parameter<double>("Ts", 0.45));
    wh_ = static_cast<float>(declare_parameter<double>("wh", 80.0));
    ws_ = static_cast<float>(declare_parameter<double>("ws", 15.0));
    wu_ = static_cast<float>(declare_parameter<double>("wu", 8.0));

    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    hard_marker_threshold_ = static_cast<float>(declare_parameter<double>("hard_marker_threshold", 0.7));
    unknown_marker_threshold_ = static_cast<float>(declare_parameter<double>("unknown_marker_threshold", 0.6));

    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
        input_map_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ObstacleLayersNode::onMap, this, std::placeholders::_1));

    map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(output_map_topic_, rclcpp::SystemDefaultsQoS());

    if (!input_cloud_topic_.empty()) {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          input_cloud_topic_, rclcpp::SensorDataQoS(),
          std::bind(&ObstacleLayersNode::onCloud, this, std::placeholders::_1));
    }

    if (publish_markers_) {
      markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          "obstacle_layers/markers", rclcpp::SystemDefaultsQoS());
    }

    RCLCPP_INFO(get_logger(), "obstacle_layers_node ready. input_map_topic='%s' output_map_topic='%s' cloud='%s'",
                input_map_topic_.c_str(), output_map_topic_.c_str(), input_cloud_topic_.c_str());
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_cloud_ = msg;
  }

  void ensurePersistentStorage(const grid_map::GridMap& map) {
    const grid_map::Size size = map.getSize();
    if (!persist_evidence_.has_value() ||
        persist_evidence_->rows() != size(0) ||
        persist_evidence_->cols() != size(1)) {
      persist_evidence_.emplace(size(0), size(1));
      persist_evidence_->setZero();
      RCLCPP_INFO(get_logger(), "Initialized persistence matrix: %dx%d", size(0), size(1));
    }
  }

  float robustLocalSupport(const grid_map::GridMap& map, const grid_map::Index& index, int radius_cells) const {
    if (!map.isValid(index, elevation_layer_)) return std::numeric_limits<float>::quiet_NaN();

    std::vector<float> zs;
    zs.reserve(static_cast<size_t>((2 * radius_cells + 1) * (2 * radius_cells + 1)));

    const bool hasVar = map.exists(variance_layer_);
    const float varMax = variance_max_;

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        grid_map::Index n(index(0) + dx, index(1) + dy);
        if (!map.isValid(n)) continue;
        if (!map.isValid(n, elevation_layer_)) continue;
        if (hasVar && map.isValid(n, variance_layer_)) {
          const float v = map.at(variance_layer_, n);
          if (!finite(v)) continue;
          if (v > varMax) continue;
        }
        const float z = map.at(elevation_layer_, n);
        if (!finite(z)) continue;
        zs.push_back(z);
      }
    }

    if (zs.size() < 3) {
      return map.at(elevation_layer_, index);
    }

    return quantile_inplace(zs, support_percentile_);
  }

  float computeSupportConfidence(const grid_map::GridMap& map, const grid_map::Index& index, int support_count, int max_support_count) const {
    float sigma_term = 0.5f;
    if (map.exists(variance_layer_) && map.isValid(index, variance_layer_)) {
      const float v = map.at(variance_layer_, index);
      if (finite(v)) {
        const float vnorm = clamp01((v - variance_min_) / std::max(1e-6f, (variance_max_ - variance_min_)));
        sigma_term = 1.0f - vnorm;
      }
    }

    const float n_term = clamp01(static_cast<float>(support_count) / std::max(1.0f, static_cast<float>(max_support_count)));
    return clamp01(0.7f * sigma_term + 0.3f * n_term);
  }

  void onMap(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    grid_map::GridMap in;
    try {
      grid_map::GridMapRosConverter::fromMessage(*msg, in);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse GridMap: %s", e.what());
      return;
    }

    if (!in.exists(elevation_layer_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input map missing elevation layer '%s'", elevation_layer_.c_str());
      return;
    }

    ensurePersistentStorage(in);

    grid_map::GridMap out = in;

    const std::string support_z_layer = "support_z";
    const std::string support_conf_layer = "support_conf";
    const std::string height_rel_layer = "height_rel";
    const std::string edge_score_layer = "edge_score";
    const std::string vertical_occ_layer = "vertical_occ";
    const std::string persist_layer = "persist_evidence";
    const std::string p_hard_layer = "p_hard";
    const std::string p_soft_layer = "p_soft";
    const std::string p_unknown_layer = "p_unknown";
    const std::string obs_cost_layer = "obs_cost";

    for (const auto& layer : {support_z_layer, support_conf_layer, height_rel_layer, edge_score_layer,
                              vertical_occ_layer, persist_layer, p_hard_layer, p_soft_layer,
                              p_unknown_layer, obs_cost_layer}) {
      if (!out.exists(layer)) {
        out.add(layer, std::numeric_limits<float>::quiet_NaN());
      }
    }

    const float res = static_cast<float>(out.getResolution());
    const double res_d = static_cast<double>(res);
    const double denom = std::max(1e-6, res_d);
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(support_window_m_ / denom)));
    const int max_support_count = (2 * radius_cells + 1) * (2 * radius_cells + 1);

    // 1) support_z + support_conf
    grid_map::Matrix& supportZ = out[support_z_layer];
    grid_map::Matrix& supportConf = out[support_conf_layer];

    for (grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);
      if (!out.isValid(idx, elevation_layer_)) {
        supportZ(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
        supportConf(idx(0), idx(1)) = 0.0f;
        continue;
      }

      // Count support samples while computing support.
      int support_count = 0;
      {
        const bool hasVar = out.exists(variance_layer_);
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            grid_map::Index n(idx(0) + dx, idx(1) + dy);
            if (!out.isValid(n)) continue;
            if (!out.isValid(n, elevation_layer_)) continue;
            if (hasVar && out.isValid(n, variance_layer_)) {
              const float v = out.at(variance_layer_, n);
              if (!finite(v)) continue;
              if (v > variance_max_) continue;
            }
            const float z = out.at(elevation_layer_, n);
            if (!finite(z)) continue;
            ++support_count;
          }
        }
      }

      supportZ(idx(0), idx(1)) = robustLocalSupport(out, idx, radius_cells);
      supportConf(idx(0), idx(1)) = computeSupportConfidence(out, idx, support_count, max_support_count);
    }

    // 2) edge_score (local discontinuity on support_z)
    grid_map::Matrix& edgeScore = out[edge_score_layer];
    for (grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);
      const float z0 = supportZ(idx(0), idx(1));
      if (!finite(z0)) {
        edgeScore(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      float maxDiff = 0.0f;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          grid_map::Index n(idx(0) + dx, idx(1) + dy);
          if (!out.isValid(n)) continue;
          const float zn = supportZ(n(0), n(1));
          if (!finite(zn)) continue;
          maxDiff = std::max(maxDiff, std::abs(z0 - zn));
        }
      }
      edgeScore(idx(0), idx(1)) = clamp01(maxDiff / std::max(1e-6f, step_hard_));
    }

    // 3) Optional point cloud features (height_rel, vertical_occ)
    grid_map::Matrix& heightRel = out[height_rel_layer];
    grid_map::Matrix& verticalOcc = out[vertical_occ_layer];

    const bool has_cloud = static_cast<bool>(last_cloud_);
    std::vector<CellReservoir> cells;
    if (has_cloud) {
      const auto& cloud = *last_cloud_;
      if (cloud.header.frame_id != out.getFrameId()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "PointCloud frame_id='%s' != map frame_id='%s' (skipping cloud; provide cloud in map frame)",
                             cloud.header.frame_id.c_str(), out.getFrameId().c_str());
      } else {
        const auto size = out.getSize();
        const int rows = size(0);
        const int cols = size(1);
        cells.resize(static_cast<size_t>(rows * cols));
        for (auto& c : cells) c.reset();

        std::minstd_rand rng(static_cast<uint32_t>(get_clock()->now().nanoseconds() & 0xffffffffu));

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
          const float x = *iter_x;
          const float y = *iter_y;
          const float z = *iter_z;
          if (!finite(x) || !finite(y) || !finite(z)) continue;

          grid_map::Position pos(x, y);
          if (!out.isInside(pos)) continue;

          grid_map::Index idx;
          if (!out.getIndex(pos, idx)) continue;

          const float sz = supportZ(idx(0), idx(1));
          if (!finite(sz)) continue;

          const int r = idx(0);
          const int c = idx(1);
          CellReservoir& cell = cells[static_cast<size_t>(r * cols + c)];

          cell.total_points++;

          const float rel = z - sz;
          if (rel > 0.0f && finite(rel)) {
            cell.observeSample(rel, rng);
          }

          if (z >= (sz + h_climb_) && z <= (sz + h_body_)) {
            cell.band_points++;
          }
        }

        for (grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
          const grid_map::Index idx(*it);
          if (!finite(supportZ(idx(0), idx(1)))) {
            heightRel(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
            verticalOcc(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
            continue;
          }
          const CellReservoir& cell = cells[static_cast<size_t>(idx(0) * cols + idx(1))];
          heightRel(idx(0), idx(1)) = cell.q90();
          verticalOcc(idx(0), idx(1)) = cell.verticalOcc();
        }
      }
    }

    if (!has_cloud || cells.empty()) {
      // If we don't have a cloud aligned, default to 0 (unknown handled later via confidence).
      for (grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
        const grid_map::Index idx(*it);
        if (!finite(supportZ(idx(0), idx(1)))) {
          heightRel(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
          verticalOcc(idx(0), idx(1)) = std::numeric_limits<float>::quiet_NaN();
          continue;
        }
        heightRel(idx(0), idx(1)) = 0.0f;
        verticalOcc(idx(0), idx(1)) = 0.0f;
      }
    }

    // 4) Probabilities + obs_cost with persistence
    grid_map::Matrix& persist = out[persist_layer];
    grid_map::Matrix& pHard = out[p_hard_layer];
    grid_map::Matrix& pSoft = out[p_soft_layer];
    grid_map::Matrix& pUnknown = out[p_unknown_layer];
    grid_map::Matrix& obsCost = out[obs_cost_layer];

    const bool hasStep = out.exists(step_layer_);
    const bool hasRough = out.exists(rough_layer_);
    const bool hasSlope = out.exists(slope_layer_);
    const bool hasNeg = out.exists(negatives_layer_);
    const bool hasClear = out.exists(clearance_layer_);

    auto& persistState = *persist_evidence_;

    for (grid_map::GridMapIterator it(out); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);

      const float conf_support = supportConf(idx(0), idx(1));
      const float sz = supportZ(idx(0), idx(1));

      if (!finite(sz)) {
        persist(idx(0), idx(1)) = 0.0f;
        pHard(idx(0), idx(1)) = 0.0f;
        pSoft(idx(0), idx(1)) = 0.0f;
        pUnknown(idx(0), idx(1)) = 1.0f;
        obsCost(idx(0), idx(1)) = wu_;
        persistState(idx(0), idx(1)) = 0.0f;
        continue;
      }

      // Normalized features (missing layers -> 0).
      float step_n = 0.0f;
      float rough_n = 0.0f;
      float slope_n = 0.0f;
      float neg_n = 0.0f;
      float clear_penalty = 0.0f;

      if (hasStep && out.isValid(idx, step_layer_)) {
        const float v = out.at(step_layer_, idx);
        if (finite(v)) step_n = clamp01(v / std::max(1e-6f, step_hard_));
      }
      if (hasRough && out.isValid(idx, rough_layer_)) {
        const float v = out.at(rough_layer_, idx);
        if (finite(v)) rough_n = clamp01(v / std::max(1e-6f, rough_hard_));
      }
      if (hasSlope && out.isValid(idx, slope_layer_)) {
        const float v = out.at(slope_layer_, idx);
        if (finite(v)) slope_n = clamp01(v / std::max(1e-6f, slope_hard_));
      }
      if (hasNeg && out.isValid(idx, negatives_layer_)) {
        const float v = out.at(negatives_layer_, idx);
        if (finite(v)) neg_n = clamp01(v / std::max(1e-6f, neg_hard_));
      }
      if (hasClear && out.isValid(idx, clearance_layer_)) {
        const float v = out.at(clearance_layer_, idx);
        if (finite(v)) clear_penalty = clamp01((clear_min_ - v) / std::max(1e-6f, clear_min_));
      }

      const float edge = finite(edgeScore(idx(0), idx(1))) ? edgeScore(idx(0), idx(1)) : 0.0f;
      const float vocc = finite(verticalOcc(idx(0), idx(1))) ? verticalOcc(idx(0), idx(1)) : 0.0f;
      const float hrel = finite(heightRel(idx(0), idx(1))) ? heightRel(idx(0), idx(1)) : 0.0f;

      const float h_soft_n = clamp01(hrel / std::max(1e-6f, h_soft_));
      const float h_hard_n = clamp01(hrel / std::max(1e-6f, h_hard_));

      const float g_hard = std::max({step_n, edge, clear_penalty, neg_n, h_hard_n});
      const float g_soft = 0.35f * slope_n + 0.35f * rough_n + 0.20f * h_soft_n + 0.10f * vocc;

      const float inst = std::max(g_hard, g_soft);
      const float prev = persistState(idx(0), idx(1));
      const float pe = alpha_ * prev + (1.0f - alpha_) * inst;
      persistState(idx(0), idx(1)) = pe;
      persist(idx(0), idx(1)) = pe;

      float conf = clamp01(0.8f * conf_support + 0.2f * pe);

      // If a lot of required layers are missing, reduce confidence (treat as more unknown).
      int missing = 0;
      missing += hasStep ? 0 : 1;
      missing += hasRough ? 0 : 1;
      missing += hasSlope ? 0 : 1;
      missing += hasNeg ? 0 : 1;
      missing += hasClear ? 0 : 1;
      if (missing >= 3) conf *= 0.7f;

      const float ph = conf * sigmoid(kh_ * (g_hard - Th_));
      const float ps = conf * (1.0f - ph) * sigmoid(ks_ * (g_soft - Ts_));
      const float pu = 1.0f - conf;

      pHard(idx(0), idx(1)) = ph;
      pSoft(idx(0), idx(1)) = ps;
      pUnknown(idx(0), idx(1)) = pu;
      obsCost(idx(0), idx(1)) = wh_ * (ph * ph) + ws_ * ps + wu_ * pu;
    }

    // 5) Publish
    auto out_msg = grid_map::GridMapRosConverter::toMessage(out);
    out_msg->header.stamp = now();
    map_pub_->publish(*out_msg);

    if (publish_markers_ && markers_pub_ && markers_pub_->get_subscription_count() > 0) {
      publishMarkers(out, pHard, pUnknown);
    }
  }

  void publishMarkers(const grid_map::GridMap& map, const grid_map::Matrix& pHard, const grid_map::Matrix& pUnknown) {
    const double res = map.getResolution();
    const auto stamp = now();

    visualization_msgs::msg::Marker hard;
    hard.header.frame_id = map.getFrameId();
    hard.header.stamp = stamp;
    hard.ns = "obstacle_layers";
    hard.id = 0;
    hard.type = visualization_msgs::msg::Marker::CUBE_LIST;
    hard.action = visualization_msgs::msg::Marker::ADD;
    hard.scale.x = res;
    hard.scale.y = res;
    hard.scale.z = 0.02;
    hard.color.r = 1.0f;
    hard.color.g = 0.0f;
    hard.color.b = 0.0f;
    hard.color.a = 0.8f;
    hard.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker unk;
    unk.header.frame_id = map.getFrameId();
    unk.header.stamp = stamp;
    unk.ns = "obstacle_layers";
    unk.id = 1;
    unk.type = visualization_msgs::msg::Marker::CUBE_LIST;
    unk.action = visualization_msgs::msg::Marker::ADD;
    unk.scale.x = res;
    unk.scale.y = res;
    unk.scale.z = 0.02;
    unk.color.r = 0.2f;
    unk.color.g = 0.2f;
    unk.color.b = 1.0f;
    unk.color.a = 0.7f;
    unk.pose.orientation.w = 1.0;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      const grid_map::Index idx(*it);
      const float ph = pHard(idx(0), idx(1));
      const float pu = pUnknown(idx(0), idx(1));
      if (!finite(ph) && !finite(pu)) continue;

      grid_map::Position p;
      map.getPosition(*it, p);
      geometry_msgs::msg::Point q;
      q.x = p.x();
      q.y = p.y();
      q.z = 0.03;

      if (finite(ph) && ph >= hard_marker_threshold_) {
        hard.points.push_back(q);
      }
      if (finite(pu) && pu >= unknown_marker_threshold_) {
        unk.points.push_back(q);
      }
    }

    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(hard);
    arr.markers.push_back(unk);
    markers_pub_->publish(arr);
  }

  // Parameters
  std::string input_map_topic_;
  std::string output_map_topic_;
  std::string input_cloud_topic_;

  std::string elevation_layer_;
  std::string variance_layer_;
  std::string slope_layer_;
  std::string step_layer_;
  std::string rough_layer_;
  std::string clearance_layer_;
  std::string negatives_layer_;

  double support_window_m_{0.45};
  float support_percentile_{0.20f};
  float variance_min_{0.01f};
  float variance_max_{0.05f};

  float step_hard_{0.25f};
  float rough_hard_{0.10f};
  float slope_hard_{0.52f};
  float neg_hard_{0.25f};
  float h_soft_{0.10f};
  float h_hard_{0.25f};
  float clear_min_{0.40f};
  float h_climb_{0.10f};
  float h_body_{0.45f};

  float alpha_{0.85f};

  float kh_{8.0f};
  float ks_{6.0f};
  float Th_{0.55f};
  float Ts_{0.45f};
  float wh_{80.0f};
  float ws_{15.0f};
  float wu_{8.0f};

  bool publish_markers_{true};
  float hard_marker_threshold_{0.7f};
  float unknown_marker_threshold_{0.6f};

  // State
  std::optional<grid_map::Matrix> persist_evidence_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;

  // ROS
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

}  // namespace obstacle_layers

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<obstacle_layers::ObstacleLayersNode>());
  rclcpp::shutdown();
  return 0;
}
