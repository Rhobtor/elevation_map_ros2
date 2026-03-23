#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>
#include <limits>

#include <grid_map_core/grid_map_core.hpp>

namespace em {
namespace layers {

// ---- helpers ----

static inline bool isFinite(float v) {
  return std::isfinite(v);
}

static inline int clampi(int v, int lo, int hi) {
  return std::max(lo, std::min(hi, v));
}

static void ensureLayer(grid_map::GridMap& map, const std::string& name) {
  if (!map.exists(name)) {
    map.add(name, std::numeric_limits<float>::quiet_NaN());
  }
}

static float readCell(const grid_map::Matrix& M, const grid_map::Index& I) {
  return M(I(0), I(1));
}

static void writeCell(grid_map::Matrix& M, const grid_map::Index& I, float v) {
  M(I(0), I(1)) = v;
}

static bool validIndex(const grid_map::GridMap& map, const grid_map::Index& I) {
  const auto size = map.getSize();
  return (I(0) >= 0 && I(1) >= 0 && I(0) < size(0) && I(1) < size(1));
}

static void neighborhoodIndicesSquare(const grid_map::GridMap& map,
                                      const grid_map::Index& center,
                                      int r,
                                      std::vector<grid_map::Index>& out) {
  out.clear();
  const auto size = map.getSize();
  const int i0 = clampi(center(0) - r, 0, size(0) - 1);
  const int i1 = clampi(center(0) + r, 0, size(0) - 1);
  const int j0 = clampi(center(1) - r, 0, size(1) - 1);
  const int j1 = clampi(center(1) + r, 0, size(1) - 1);
  out.reserve((i1 - i0 + 1) * (j1 - j0 + 1));
  for (int i = i0; i <= i1; ++i) {
    for (int j = j0; j <= j1; ++j) {
      out.emplace_back(grid_map::Index(i, j));
    }
  }
}

// ---- API ----

void addSlope(grid_map::GridMap& map, double /*cell*/) {
  const std::string in  = "elevation";
  const std::string out = "slope";
  if (!map.exists(in)) return;
  ensureLayer(map, out);

  auto& Mout = map[out];
  const auto& Min = map[in];
  const double res = map.getResolution();
  const auto size = map.getSize();

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const int i = I(0), j = I(1);

    auto val_c = Min(i, j);
    if (!isFinite(val_c)) { Mout(i, j) = std::numeric_limits<float>::quiet_NaN(); continue; }

    // Vecinos para derivadas (centrales cuando se puede)
    const int im1 = clampi(i - 1, 0, size(0) - 1);
    const int ip1 = clampi(i + 1, 0, size(0) - 1);
    const int jm1 = clampi(j - 1, 0, size(1) - 1);
    const int jp1 = clampi(j + 1, 0, size(1) - 1);

    const float left  = Min(im1, j);
    const float right = Min(ip1, j);
    const float down  = Min(i, jm1);
    const float up    = Min(i, jp1);

    // Si vecinos NaN, degradar a forward/backward cuando sea posible
    double dzdx = 0.0, dzdy = 0.0;
    if (isFinite(left) && isFinite(right)) {
      dzdx = (right - left) / (2.0 * res);
    } else if (isFinite(right)) {
      dzdx = (right - val_c) / res;
    } else if (isFinite(left)) {
      dzdx = (val_c - left) / res;
    } else {
      Mout(i, j) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    if (isFinite(down) && isFinite(up)) {
      dzdy = (up - down) / (2.0 * res);
    } else if (isFinite(up)) {
      dzdy = (up - val_c) / res;
    } else if (isFinite(down)) {
      dzdy = (val_c - down) / res;
    } else {
      Mout(i, j) = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    const double tan_theta = std::sqrt(dzdx * dzdx + dzdy * dzdy);
    Mout(i, j) = static_cast<float>(std::atan(tan_theta));  // rad
  }
}

void addRoughness(grid_map::GridMap& map, double window_m) {
  const std::string in  = "elevation";
  const std::string out = "rough";
  if (!map.exists(in)) return;
  ensureLayer(map, out);

  auto& Mout = map[out];
  const auto& Min = map[in];
  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(window_m / res)));

  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    neighborhoodIndicesSquare(map, I, r, neigh);

    double sum = 0.0, sum2 = 0.0;
    int n = 0;
    for (const auto& N : neigh) {
      const float v = readCell(Min, N);
      if (!isFinite(v)) continue;
      sum += v; sum2 += (double)v * v; ++n;
    }
    if (n < 3) {
      writeCell(Mout, I, std::numeric_limits<float>::quiet_NaN());
      continue;
    }
    const double mean = sum / n;
    const double var  = std::max(0.0, (sum2 / n) - mean * mean);
    writeCell(Mout, I, static_cast<float>(std::sqrt(var))); // m (std)
  }
}

void addStep(grid_map::GridMap& map, double window_m) {
  const std::string in  = "elevation";
  const std::string out = "step";
  if (!map.exists(in)) return;
  ensureLayer(map, out);

  auto& Mout = map[out];
  const auto& Min = map[in];
  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(window_m / res)));

  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    neighborhoodIndicesSquare(map, I, r, neigh);

    float vmin =  std::numeric_limits<float>::infinity();
    float vmax = -std::numeric_limits<float>::infinity();
    int n = 0;
    for (const auto& N : neigh) {
      const float v = readCell(Min, N);
      if (!isFinite(v)) continue;
      vmin = std::min(vmin, v);
      vmax = std::max(vmax, v);
      ++n;
    }
    if (n < 2) {
      writeCell(Mout, I, std::numeric_limits<float>::quiet_NaN());
      continue;
    }
    writeCell(Mout, I, (isFinite(vmin) && isFinite(vmax)) ? (vmax - vmin) : std::numeric_limits<float>::quiet_NaN());
  }
}

void addObstacleBinaryFromGeom(grid_map::GridMap& map,
                               double slope_thresh_rad,
                               double step_thresh_m) {
  ensureLayer(map, "obstacles");
  auto& Mobst = map["obstacles"];

  const bool haveSlope = map.exists("slope");
  const bool haveStep  = map.exists("step");
  if (!haveSlope && !haveStep) {
    // nada que combinar
    return;
  }

  const grid_map::Matrix* Ms = haveSlope ? &map["slope"] : nullptr;
  const grid_map::Matrix* Mt = haveStep  ? &map["step"]  : nullptr;

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    bool is_obst = false;

    if (Ms) {
      const float s = readCell(*Ms, I);
      if (isFinite(s) && s > static_cast<float>(slope_thresh_rad)) is_obst = true;
    }
    if (Mt) {
      const float st = readCell(*Mt, I);
      if (isFinite(st) && st > static_cast<float>(step_thresh_m)) is_obst = true;
    }

    writeCell(Mobst, I, is_obst ? 1.0f : 0.0f);
  }
}

// ---- IMPLEMENTACIÓN BÁSICA QUE FALTABA ----
// Detecta negativos comparando el centro contra la media de un anillo cuadrado de radio 'ring_m'.
// Escribe en "negatives" (0/1). Usa capa "elevation".
void addNegativeObstacles(grid_map::GridMap& map,
                          double drop_thresh_m,
                          double ring_m) {
  const std::string inElev = "elevation";
  const std::string outNeg = "negatives";

  if (!map.exists(inElev)) return;
  ensureLayer(map, outNeg);

  const auto& Me = map[inElev];
  auto& Mn = map[outNeg];

  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(ring_m / res)));
  if (r < 1) return;

  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const float c = readCell(Me, I);
    if (!isFinite(c)) { writeCell(Mn, I, std::numeric_limits<float>::quiet_NaN()); continue; }

    neighborhoodIndicesSquare(map, I, r, neigh);

    double sum = 0.0; int n = 0;
    for (const auto& N : neigh) {
      // evitar el propio centro
      if (N(0) == I(0) && N(1) == I(1)) continue;

      const float v = readCell(Me, N);
      if (!isFinite(v)) continue;
      sum += v;
      ++n;
    }

    if (n < 6) { writeCell(Mn, I, 0.0f); continue; } // poco contexto => conservador
    const double mean_ring = sum / std::max(1, n);
    const bool neg = (mean_ring - c) > drop_thresh_m;
    writeCell(Mn, I, neg ? 1.0f : 0.0f);
  }
}

// Variante robusta (parámetros extra y nombres de capas)
void addNegativeObstaclesRobust(grid_map::GridMap& map,
                                double drop_thresh_m,
                                double ring_m,
                                double min_valid_ratio,
                                double slope_gate_rad,
                                const std::string& elevation_layer,
                                const std::string& slope_layer,
                                const std::string& out_layer) {
  const std::string inElev = elevation_layer.empty() ? "elevation" : elevation_layer;
  const std::string inSlope= slope_layer.empty()     ? "slope"     : slope_layer;
  const std::string outNeg = out_layer.empty()       ? "negatives" : out_layer;

  if (!map.exists(inElev)) return;
  ensureLayer(map, outNeg);

  // Si no hay slope, intenta calcular algo razonable antes de filtrar
  if (!map.exists(inSlope)) {
    // cálculo local sencillo
    addSlope(map, map.getResolution());
  }

  const auto& Me = map[inElev];
  const auto& Ms = map[inSlope];
  auto& Mn = map[outNeg];

  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(ring_m / res)));
  if (r < 1) return;

  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const float c = readCell(Me, I);
    const float s0= readCell(Ms, I);

    if (!isFinite(c)) { writeCell(Mn, I, std::numeric_limits<float>::quiet_NaN()); continue; }
    if (isFinite(s0) && s0 > static_cast<float>(slope_gate_rad)) {
      // Puerta: si ya hay mucha pendiente local, no marcar negativo (probable borde/escalón)
      writeCell(Mn, I, 0.0f); continue;
    }

    neighborhoodIndicesSquare(map, I, r, neigh);

    double sum = 0.0; int n = 0, nvalid = 0;
    for (const auto& N : neigh) {
      // evitar el propio centro
      if (N(0) == I(0) && N(1) == I(1)) continue;

      const float v = readCell(Me, N);
      if (!isFinite(v)) continue;
      sum += v;
      ++n;
      ++nvalid;
    }


    const int total_ring = static_cast<int>(neigh.size()) - 1;
    const double ratio = total_ring > 0 ? static_cast<double>(nvalid) / total_ring : 0.0;
    if (ratio < min_valid_ratio) { writeCell(Mn, I, 0.0f); continue; }

    if (n < 6) { writeCell(Mn, I, 0.0f); continue; }
    const double mean_ring = sum / std::max(1, n);
    const bool neg = (mean_ring - c) > drop_thresh_m;
    writeCell(Mn, I, neg ? 1.0f : 0.0f);
  }
}

void addCvarTraversability(grid_map::GridMap& map,
                           double alpha,
                           double window_m /*, const std::string& out = "cvar_risk"*/) {
  const std::string in = "slope";
  const std::string out= "cvar_risk";
  if (!map.exists(in)) {
    // si no hay slope, créalo rápido
    addSlope(map, map.getResolution());
    if (!map.exists(in)) return;
  }
  ensureLayer(map, out);

  auto& Mrisk = map[out];
  const auto& Mslope = map[in];

  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(window_m / res)));
  const double alpha_clamped = std::min(0.999, std::max(0.001, alpha));

  std::vector<grid_map::Index> neigh;
  std::vector<float> vals;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    neighborhoodIndicesSquare(map, I, r, neigh);

    vals.clear();
    vals.reserve(neigh.size());
    for (const auto& N : neigh) {
      const float s = readCell(Mslope, N);
      if (isFinite(s)) vals.push_back(s);
    }
    if (vals.size() < 6) {
      writeCell(Mrisk, I, std::numeric_limits<float>::quiet_NaN());
      continue;
    }

    std::sort(vals.begin(), vals.end());
    const size_t k = static_cast<size_t>(std::floor(alpha_clamped * (vals.size() - 1)));
    const float VaR = vals[k];

    double tail_sum = 0.0; size_t tail_n = 0;
    for (size_t t = k; t < vals.size(); ++t) { tail_sum += vals[t]; ++tail_n; }
    const double cvar = tail_n ? (tail_sum / tail_n) : VaR;

    // normalización simple por pi/2 (pendiente máxima teórica)
    const double risk = std::min(1.0, std::max(0.0, cvar / (M_PI_2)));
    writeCell(Mrisk, I, static_cast<float>(risk));
  }
}

void combineToFinalObstacles(grid_map::GridMap& map,
                             const std::string& obstacles_layer,
                             const std::string& negatives_layer,
                             const std::string& cvar_layer,
                             double cvar_tau,
                             const std::string& out_layer) {
  ensureLayer(map, out_layer);
  auto& Mout = map[out_layer];

  const grid_map::Matrix* Mo = map.exists(obstacles_layer) ? &map[obstacles_layer] : nullptr;
  const grid_map::Matrix* Mn = map.exists(negatives_layer) ? &map[negatives_layer] : nullptr;
  const grid_map::Matrix* Mc = map.exists(cvar_layer)      ? &map[cvar_layer]      : nullptr;

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    bool occ = false;

    if (Mo) {
      const float v = readCell(*Mo, I);
      if (isFinite(v) && v >= 0.5f) occ = true;
    }
    if (Mn && !occ) {
      const float v = readCell(*Mn, I);
      if (isFinite(v) && v >= 0.5f) occ = true;
    }
    if (Mc && !occ) {
      const float v = readCell(*Mc, I);
      if (isFinite(v) && v >= static_cast<float>(cvar_tau)) occ = true;
    }

    writeCell(Mout, I, occ ? 1.0f : 0.0f);
  }
}







/**
 * Crea una capa binaria "grid" (0/1) de celdas conocidas (elevation no NaN)
 * Opcionalmente, si gate_by_variance=true, filtra también por varianza < var_thresh.
 */

void addGridKnown(grid_map::GridMap& map,
                  bool gate_by_variance,
                  double var_thresh,
                  const std::string& elevation_layer = "elevation",
                  const std::string& variance_layer  = "variance",
                  const std::string& out_layer       = "grid") {
  if (!map.exists(elevation_layer)) return;
  ensureLayer(map, out_layer);

  const auto& Me = map[elevation_layer];
  const bool haveVar = gate_by_variance && map.exists(variance_layer);
  const grid_map::Matrix* Mv = haveVar ? &map[variance_layer] : nullptr;

  auto& Mg = map[out_layer];

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const float h = readCell(Me, I);
    if (!isFinite(h)) { writeCell(Mg, I, 0.0f); continue; }

    if (haveVar) {
      const float v = readCell(*Mv, I);
      if (!isFinite(v) || v > static_cast<float>(var_thresh)) {
        writeCell(Mg, I, 0.0f); // conocido pero poco confiable -> trata como desconocido
        continue;
      }
    }
    writeCell(Mg, I, 1.0f); // conocido
  }
}

/**
 * Calcula "frontier" = celdas conocidas (grid==1) que tocan alguna celda desconocida (grid==0).
 * edge_is_unknown: si true, los bordes fuera de índice cuentan como desconocidos -> marcan frontera.
 */
void addFrontierFromGrid(grid_map::GridMap& map,
                         const std::string& grid_layer   = "grid",
                         const std::string& out_layer    = "frontier",
                         bool edge_is_unknown            = true) {
  if (!map.exists(grid_layer)) return;
  ensureLayer(map, out_layer);

  const auto& Mg = map[grid_layer];
  auto& Mf = map[out_layer];

  const auto size = map.getSize();
  const int H = size(0), W = size(1);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const int i = I(0), j = I(1);

    const float g = readCell(Mg, I);
    if (!isFinite(g) || g < 0.5f) { writeCell(Mf, I, 0.0f); continue; }

    bool is_frontier = false;
    for (int di = -1; di <= 1 && !is_frontier; ++di) {
      for (int dj = -1; dj <= 1 && !is_frontier; ++dj) {
        if (di == 0 && dj == 0) continue;
        const int ni = i + di;
        const int nj = j + dj;
        if (ni < 0 || nj < 0 || ni >= H || nj >= W) {
          if (edge_is_unknown) is_frontier = true;
          continue;
        }
        const grid_map::Index N(ni, nj);
        const float gn = readCell(Mg, N);
        if (!isFinite(gn) || gn < 0.5f) {
          is_frontier = true;
        }
      }
    }

    writeCell(Mf, I, is_frontier ? 1.0f : 0.0f);
  }
}
/**
 * Convierte una capa de ocupación estilo OccupancyGrid (unknown=-1, free=0, occupied=100)
 * a una máscara binaria (unknown=NaN, free=0, occupied=1).
 */
void occupancyLikeToMask(grid_map::GridMap& map,
                         const std::string& src_layer,
                         const std::string& dst_layer) {
  if (!map.exists(src_layer)) return;
  ensureLayer(map, dst_layer);
  const auto& S = map[src_layer];
  auto& D = map[dst_layer];
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const auto I = grid_map::Index(*it);
    const float v = readCell(S, I);
    if (!std::isfinite(v) || v < 0.0f) {
      writeCell(D, I, std::numeric_limits<float>::quiet_NaN()); // unknown
    } else if (v >= 50.0f) {
      writeCell(D, I, 1.0f);  // occupied
    } else {
      writeCell(D, I, 0.0f);  // free
    }
  }
}
/**
 * (Opcional) Si ya combinas obstáculos en, p.ej., "final_obstacles" (0/1),
 * genera una ocupación 3-estados estilo OccupancyGrid:
 *   unknown = -1, free = 0, occupied = 100  (en float)
 */
void addOccupancyLike(grid_map::GridMap& map,
                      const std::string& grid_known_layer = "grid",
                      const std::string& obstacles_layer  = "final_obstacles",
                      const std::string& out_layer        = "occupancy_like") {
  if (!map.exists(grid_known_layer)) return;
  ensureLayer(map, out_layer);

  const auto& Mk = map[grid_known_layer];
  const grid_map::Matrix* Mo = map.exists(obstacles_layer) ? &map[obstacles_layer] : nullptr;
  auto& Mout = map[out_layer];

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const float k = readCell(Mk, I);

    if (!isFinite(k) || k < 0.5f) {
      writeCell(Mout, I, -1.0f); // desconocido
      continue;
    }
    bool occ = false;
    if (Mo) {
      const float o = readCell(*Mo, I);
      occ = (isFinite(o) && o >= 0.5f);
    }
    writeCell(Mout, I, occ ? 100.0f : 0.0f);
  }
}

static inline float norm01(float v, double vmin, double vmax) {
  if (!std::isfinite(v)) return std::numeric_limits<float>::quiet_NaN();
  if (vmax <= vmin) return std::numeric_limits<float>::quiet_NaN();
  double x = (v - vmin) / (vmax - vmin);
  if (x < 0.0) x = 0.0;
  if (x > 1.0) x = 1.0;
  return static_cast<float>(x);
}

// Vecindario circular (disco) de radio r (en celdas)
static void neighborhoodIndicesDisk(const grid_map::GridMap& map,
                                    const grid_map::Index& center,
                                    int r,
                                    std::vector<grid_map::Index>& out) {
  out.clear();
  const auto size = map.getSize();
  const int i0 = clampi(center(0) - r, 0, size(0) - 1);
  const int i1 = clampi(center(0) + r, 0, size(0) - 1);
  const int j0 = clampi(center(1) - r, 0, size(1) - 1);
  const int j1 = clampi(center(1) + r, 0, size(1) - 1);
  const int r2 = r * r;
  for (int i = i0; i <= i1; ++i) {
    for (int j = j0; j <= j1; ++j) {
      const int di = i - center(0);
      const int dj = j - center(1);
      if (di*di + dj*dj <= r2) out.emplace_back(grid_map::Index(i, j));
    }
  }
}



/**
 * Suma ponderada de capas de coste normalizadas -> out_layer (e.g., "multi_cost").
 * - layers[k] se normaliza con minmax[k] a [0,1] y se multiplica por weights[k].
 * - Si ninguna capa es válida en una celda: escribe NaN.
 */
void addMultiCost(grid_map::GridMap& map,
                  const std::vector<std::string>& layers,
                  const std::vector<double>& weights,
                  const std::vector<std::pair<double,double>>& minmax,
                  const std::string& out_layer = "multi_cost") {
  if (layers.empty() || layers.size() != weights.size() || layers.size() != minmax.size()) return;
  for (const auto& L : layers) if (!map.exists(L)) return;

  ensureLayer(map, out_layer);
  auto& Mout = map[out_layer];

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    double acc = 0.0; bool any = false;
    for (size_t k = 0; k < layers.size(); ++k) {
      const auto& M = map[layers[k]];
      const float v = readCell(M, I);
      const float n = norm01(v, minmax[k].first, minmax[k].second);
      if (std::isfinite(n)) {
        acc += weights[k] * static_cast<double>(n);
        any = true;
      }
    }
    writeCell(Mout, I, any ? static_cast<float>(acc) : std::numeric_limits<float>::quiet_NaN());
  }
}

/**
 * Umbraliza una capa de coste -> máscara binaria no_go (0/1).
 * Opcionalmente dilata con un radio en metros para margen de seguridad.
 */
void addNoGoFromCost(grid_map::GridMap& map,
                     const std::string& cost_layer,
                     double tau,
                     double inflate_radius_m,
                     const std::string& out_layer = "no_go") {
  if (!map.exists(cost_layer)) return;
  ensureLayer(map, out_layer);

  const auto& Mc = map[cost_layer];
  auto& Mng = map[out_layer];

  const double res = map.getResolution();
  const int r = (inflate_radius_m > 0.0) ? std::max(1, static_cast<int>(std::round(inflate_radius_m / res))) : 0;

  // Paso 1: máscara base
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const float c = readCell(Mc, I);
    writeCell(Mng, I, (std::isfinite(c) && c >= static_cast<float>(tau)) ? 1.0f : 0.0f);
  }

  if (r <= 0) return;

  // Paso 2: dilatación circular (margen)
  grid_map::Matrix Mcopy = Mng; // copia
  std::vector<grid_map::Index> neigh;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    if (readCell(Mcopy, I) >= 0.5f) {
      neighborhoodIndicesDisk(map, I, r, neigh);
      for (const auto& N : neigh) writeCell(Mng, N, 1.0f);
    }
  }
}

/**
 * Filtra "frontier" usando "no_go": mantiene frontera solo donde no_go==0
 * y además a cierta "clearance_m" (dilatación de no_go).
 * Escribe resultado en out_layer (e.g., "frontier_ok").
 */
void filterFrontierByNoGo(grid_map::GridMap& map,
                          const std::string& frontier_layer = "frontier",
                          const std::string& no_go_layer    = "no_go",
                          double clearance_m                 = 0.0,
                          const std::string& out_layer       = "frontier_ok") {
  if (!map.exists(frontier_layer) || !map.exists(no_go_layer)) return;
  ensureLayer(map, out_layer);

  const auto& Mf = map[frontier_layer];
  const auto& Mng = map[no_go_layer];
  auto& Mout = map[out_layer];

  const double res = map.getResolution();
  const int r = (clearance_m > 0.0) ? std::max(1, static_cast<int>(std::round(clearance_m / res))) : 0;

  // Precalcular no_go dilatado a "clearance"
  grid_map::Matrix Mblk = Mng;
  if (r > 0) {
    grid_map::Matrix base = Mng;
    std::vector<grid_map::Index> neigh;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      const grid_map::Index I(*it);
      if (readCell(base, I) >= 0.5f) {
        neighborhoodIndicesDisk(map, I, r, neigh);
        for (const auto& N : neigh) writeCell(Mblk, N, 1.0f);
      }
    }
  }

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    const float fr = readCell(Mf, I);
    const float blk= readCell(Mblk, I);
    const bool keep = (std::isfinite(fr) && fr >= 0.5f) && !(std::isfinite(blk) && blk >= 0.5f);
    writeCell(Mout, I, keep ? 1.0f : 0.0f);
  }
}

/**
 * Estampa vetos ("no se puede ir por aquí") en 'no_go' a partir de índices
 * o posiciones (elige una de las dos sobrecargas). Útil para feedback del planificador.
 */
void stampNoGoAtIndices(grid_map::GridMap& map,
                        const std::vector<grid_map::Index>& inds,
                        double radius_m,
                        const std::string& no_go_layer = "no_go") {
  ensureLayer(map, no_go_layer);
  auto& Mng = map[no_go_layer];
  const double res = map.getResolution();
  const int r = std::max(1, static_cast<int>(std::round(radius_m / res)));

  std::vector<grid_map::Index> neigh;
  for (const auto& I : inds) {
    if (!validIndex(map, I)) continue;
    neighborhoodIndicesDisk(map, I, r, neigh);
    for (const auto& N : neigh) writeCell(Mng, N, 1.0f);
  }
}

void stampNoGoAtPositions(grid_map::GridMap& map,
                          const std::vector<grid_map::Position>& poss,
                          double radius_m,
                          const std::string& no_go_layer = "no_go") {
  std::vector<grid_map::Index> inds; inds.reserve(poss.size());
  for (const auto& p : poss) {
    grid_map::Index I;
    if (map.getIndex(p, I)) inds.push_back(I);
  }
  stampNoGoAtIndices(map, inds, radius_m, no_go_layer);
}

/**
 * Decaimiento de una máscara binaria/continua (e.g., no_go) para que el veto
 * no sea permanente. Aplica: v = max(0, v - rate*dt).
 */
void decayLayer(grid_map::GridMap& map,
                const std::string& layer,
                double rate_per_sec,
                double dt_sec) {
  if (!map.exists(layer)) return;
  auto& M = map[layer];
  const float d = static_cast<float>(std::max(0.0, rate_per_sec * dt_sec));

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index I(*it);
    float v = readCell(M, I);
    if (!std::isfinite(v)) continue;
    v = std::max(0.0f, v - d);
    writeCell(M, I, v);
  }
}



// void addNoGoHard(grid_map::GridMap& map,
//                  double slope_blocking_rad,
//                  double rough_blocking_m,
//                  bool use_obstacles,
//                  bool use_negatives,
//                  bool use_cvar,
//                  double cvar_tau,
//                  double inflate_radius_m,
//                  const std::string& out_layer) {
//   ensureLayer(map, out_layer);
//   auto& Mout = map[out_layer];

//   const grid_map::Matrix* Ms = map.exists("slope")        ? &map["slope"]        : nullptr;
//   const grid_map::Matrix* Mr = map.exists("rough")        ? &map["rough"]        : nullptr;
//   const grid_map::Matrix* Mo = (use_obstacles && map.exists("obstacles")) ? &map["obstacles"] : nullptr;
//   const grid_map::Matrix* Mn = (use_negatives  && map.exists("negatives")) ? &map["negatives"] : nullptr;
//   const grid_map::Matrix* Mc = (use_cvar       && map.exists("cvar_risk")) ? &map["cvar_risk"] : nullptr;

//   ensureLayer(map, out_layer);
//   auto& Mout = map[out_layer];
//   ensureLayer(map, "no_go_why");               // <- nueva capa de debug
//   auto& Mwhy = map["no_go_why"];            // 0=libre, 1=obst, 2=neg, 3=slope, 4=rough, 5=cvar

//   for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//     const grid_map::Index I(*it);
//     bool occ = false;

//     if (Mo) { const float v = readCell(*Mo, I); occ = occ || (std::isfinite(v) && v >= 0.5f); }
//     if (Mn) { const float v = readCell(*Mn, I); occ = occ || (std::isfinite(v) && v >= 0.5f); }
//     if (Ms) { const float v = readCell(*Ms, I); occ = occ || (std::isfinite(v) && v >  static_cast<float>(slope_blocking_rad)); }
//     if (Mr) { const float v = readCell(*Mr, I); occ = occ || (std::isfinite(v) && v >  static_cast<float>(rough_blocking_m)); }
//     if (Mc) { const float v = readCell(*Mc, I); occ = occ || (std::isfinite(v) && v >= static_cast<float>(cvar_tau)); }

//     writeCell(Mout, I, occ ? 1.0f : 0.0f);
//   }

//   // Inflado opcional (margen de seguridad)
//   if (inflate_radius_m > 0.0) {
//     const int r = std::max(1, static_cast<int>(std::round(inflate_radius_m / map.getResolution())));
//     grid_map::Matrix copy = Mout;
//     std::vector<grid_map::Index> neigh;
//     for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//       const grid_map::Index I(*it);
//       if (readCell(copy, I) >= 0.5f) {
//         neighborhoodIndicesDisk(map, I, r, neigh);
//         for (const auto& N : neigh) writeCell(Mout, N, 1.0f);
//       }
//     }
//   }
// }


} // namespace layers
} // namespace em
