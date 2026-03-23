#pragma once
#include <grid_map_core/grid_map_core.hpp>
#include <vector>
#include <string>

namespace em::layers {


void addSlope(grid_map::GridMap& map, double h);
void addRoughness(grid_map::GridMap& map, double window_m);
void addStep(grid_map::GridMap& map, double window_m);
void addObstacleBinaryFromGeom(grid_map::GridMap& map,
                               double slope_thresh_rad,
                               double step_thresh_m);
void addTraversabilityFromSlopeRough(grid_map::GridMap& map,
                                     double slope_max_rad,
                                     double rough_max_m,
                                     double w_slope,
                                     double w_rough);
void addCvarTraversability(grid_map::GridMap& map,
                           double alpha,
                           double window_m);



void addNegativeObstacles(grid_map::GridMap& map,
                          double drop_thresh_m,
                          double ring_m);

void addNegativeObstaclesRobust(grid_map::GridMap& map,
                                double drop_thresh_m,
                                double ring_m,
                                double min_valid_ratio,
                                double slope_gate_rad,
                                const std::string& elevation_layer,
                                const std::string& slope_layer,
                                const std::string& out_layer);


void combineToFinalObstacles(grid_map::GridMap& map,
                             const std::string& obstacles_layer,
                             const std::string& negatives_layer,
                             const std::string& cvar_layer,
                             double cvar_tau,
                             const std::string& out_layer);

void addGridKnown(grid_map::GridMap& map,
                  bool gate_by_variance,
                  double var_thresh,
                  const std::string& elevation_layer = "elevation",
                  const std::string& variance_layer  = "variance",
                  const std::string& out_layer       = "grid");

void addFrontierFromGrid(grid_map::GridMap& map,
                         const std::string& grid_layer = "grid",
                         const std::string& out_layer  = "frontier",
                         bool edge_is_unknown          = true);



void addNoGoHard(grid_map::GridMap& map,
                 double slope_blocking_rad,
                 double rough_blocking_m,
                 bool use_obstacles,
                 bool use_negatives,
                 bool use_cvar,
                 double cvar_tau,
                 double inflate_radius_m,
                 const std::string& out_layer = "no_go");

void occupancyLikeToMask(grid_map::GridMap& map,
                         const std::string& src_layer = "occupancy_like",
                         const std::string& dst_layer = "occupancy_mask");

void addOccupancyLike(grid_map::GridMap& map,
                      const std::string& grid_known_layer = "grid",
                      const std::string& obstacles_layer  = "final_obstacles",
                      const std::string& out_layer        = "occupancy_like");

// ===== multi-cost + no-go + filtros =====
void addMultiCost(grid_map::GridMap& map,
                  const std::vector<std::string>& layers,
                  const std::vector<double>& weights,
                  const std::vector<std::pair<double,double>>& minmax,
                  const std::string& out_layer = "multi_cost");

void addNoGoFromCost(grid_map::GridMap& map,
                     const std::string& cost_layer,
                     double tau,
                     double inflate_radius_m,
                     const std::string& out_layer = "no_go");

void filterFrontierByNoGo(grid_map::GridMap& map,
                          const std::string& frontier_layer = "frontier",
                          const std::string& no_go_layer    = "no_go",
                          double clearance_m                 = 0.0,
                          const std::string& out_layer       = "frontier_ok");

void stampNoGoAtIndices(grid_map::GridMap& map,
                        const std::vector<grid_map::Index>& inds,
                        double radius_m,
                        const std::string& no_go_layer = "no_go");

void stampNoGoAtPositions(grid_map::GridMap& map,
                          const std::vector<grid_map::Position>& poss,
                          double radius_m,
                          const std::string& no_go_layer = "no_go");

void decayLayer(grid_map::GridMap& map,
                const std::string& layer,
                double rate_per_sec,
                double dt_sec);

} // namespace em::layers
