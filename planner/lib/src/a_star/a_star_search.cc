#include "a_star/a_star_search.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using std::cout;
using std::endl;

// 9 neighbors in 2d
static std::vector<Eigen::Vector2i> kNeighbors = std::vector<Eigen::Vector2i>{
    Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, 1),
    Eigen::Vector2i(0, -1),  Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, -1),
    Eigen::Vector2i(1, 0),   Eigen::Vector2i(1, 1),
};

void Astar::Init(const double cost_threshold, const int num_layers,
                 const double resolution,  const double step_cost_weight, const Eigen::MatrixXd& cost_map,
                 const Eigen::MatrixXd& height_map,
                 const Eigen::MatrixXd& ele_map) {
  auto t0 = std::chrono::high_resolution_clock::now();
  cost_threshold_ = cost_threshold;
step_cost_weight_  = step_cost_weight;

  max_x_ = cost_map.cols();
  max_y_ = cost_map.rows() / num_layers;
  max_layers_ = num_layers;
  xy_size_ = max_x_ * max_y_;

  int row_offset = 0;
  grid_map_.resize(max_layers_);
  for (size_t i = 0; i < max_layers_; ++i) {
    row_offset = i * max_y_;
    grid_map_[i].resize(max_y_);
    for (size_t j = 0; j < max_y_; ++j) {
      grid_map_[i][j].resize(max_x_);
      for (size_t k = 0; k < max_x_; ++k) {
        double height = height_map(j + row_offset, k);
        // 使用四舍五入而非截断，避免微小高度差异导致z值跳变影响路径规划
        double z = std::round(height / resolution);
        grid_map_[i][j][k] = Node(Eigen::Vector3i(z, j, k), nullptr);
        grid_map_[i][j][k].cost = cost_map(j + row_offset, k);
        grid_map_[i][j][k].height = height;
        grid_map_[i][j][k].ele = ele_map(j + row_offset, k);
        grid_map_[i][j][k].layer = i;
        // 标记节点是否有有效地面数据（height < -50表示NaN/无效）
        grid_map_[i][j][k].valid = (height > -50.0);
      }
    }
  }
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);

  search_layers_offset_.clear();
  search_layers_offset_.emplace_back(0);
  for (int i = 0; i < search_layer_depth_; ++i) {
    search_layers_offset_.emplace_back(-(i + 1));
    search_layers_offset_.emplace_back(i + 1);
  }

  printf(
      "Astar initialized, max_x: %d, max_y: %d, max_layers: %d, time elapsed: "
      "%f ms\n",
      max_x_, max_y_, max_layers_, duration.count() / 1000.0);
}

void Astar::Reset() {
  for (size_t i = 0; i < grid_map_.size(); ++i) {
    for (size_t j = 0; j < grid_map_[i].size(); ++j) {
      for (size_t k = 0; k < grid_map_[i][j].size(); ++k) {
        grid_map_[i][j][k].Reset();
      }
    }
  }
}

int Astar::GetHash(const Eigen::Vector3i& idx) const {
  return idx[0] * 10000000 + idx[1] * max_x_ + idx[2];
}

bool Astar::Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) {
  auto t0 = std::chrono::high_resolution_clock::now();

  if (!search_result_.empty()) {
    Reset();
    search_result_.clear();
  }

  auto start_node = &grid_map_[start[0]][start[2]][start[1]];
  auto goal_node = &grid_map_[goal[0]][goal[2]][goal[1]];
  
  printf("[DEBUG] Start node: layer=%d, row=%d, col=%d, height=%.2f, valid=%d\n",
         start[0], start[1], start[2], start_node->height, start_node->valid);
  printf("[DEBUG] Goal node: layer=%d, row=%d, col=%d, height=%.2f, valid=%d\n",
         goal[0], goal[1], goal[2], goal_node->height, goal_node->valid);
  
  start_node->g = 0.0;

  if (goal_node->cost > cost_threshold_) {
    printf("goal node is not reachable, cost: %f", goal_node->cost);
    return false;
  }

  std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
  std::unordered_map<int, Node*> closed_set;

  open_set.push(start_node);

  printf("start searching\n");

  while (!open_set.empty()) {
    Node* current_node = open_set.top();
    open_set.pop();
    
    // 跳过无效节点
    if (!current_node->valid) {
      if (debug_) {
        printf("[DEBUG] Skipping invalid node: layer=%d, row=%d, col=%d, height=%.2f\n",
               current_node->layer, current_node->idx[1], current_node->idx[2], current_node->height);
      }
      continue;
    }
    
    // 关键修复：跳过已经在closed_set中的节点（这些是过期的副本）
    // 当同一个节点被多次添加到open_set时，只处理第一次（g值最小的那次）
    auto existing = closed_set.find(GetHash(current_node->idx));
    if (existing != closed_set.end()) {
      continue;  // 已经处理过，跳过这个过期副本
    }

    if (current_node->idx == goal_node->idx) {
      int path_idx = 0;
      while (current_node->parent != nullptr) {
        // search_result_.emplace_back(Eigen::Vector3i(
        //     current_node->layer, current_node->idx[1],
        //     current_node->idx[2]));
        search_result_.emplace_back(current_node);
        
        // 打印前10个路径节点的信息
        if (path_idx < 10) {
          printf("[DEBUG] Path node %d: layer=%d, row=%d, col=%d, height=%.2f, valid=%d\n",
                 path_idx, current_node->layer, current_node->idx[1], current_node->idx[2],
                 current_node->height, current_node->valid);
        }
        
        current_node = current_node->parent;
        path_idx++;
      }
      std::reverse(search_result_.begin(), search_result_.end());
      if (debug_) ConvertClosedSetToMatrix(closed_set);
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - t0);
      printf("path found, time elapsed: %f ms\n",
             duration.count() / 1000.0);
      return true;
    }

    closed_set[GetHash(current_node->idx)] = current_node;

    // 修复：只在当前层无效或需要gateway切换时才调用DecideLayer
    // 这避免了在平面区域频繁切换层导致的绕路问题
    int layer = current_node->layer;
    
    // 检查当前层在当前位置是否有效
    int i_check = current_node->idx[1];
    int j_check = current_node->idx[2];
    const Node& current_pos_node = grid_map_[layer][i_check][j_check];
    
    // 只在以下情况调用DecideLayer:
    // 1. 当前层在当前位置无效
    // 2. 当前位置有gateway标记（需要切换层）
    bool need_layer_change = !current_pos_node.valid || 
                             std::abs(current_pos_node.ele) > 0.5;
    
    if (need_layer_change) {
      layer = DecideLayer(current_node);
    }

    int i, j = 0;
    double tentative_g = 0.0;
    for (const auto& neighbor : kNeighbors) {
      i = current_node->idx[1] + neighbor[0];
      j = current_node->idx[2] + neighbor[1];

      if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_) {
        continue;
      }

      auto neighbor_node = &grid_map_[layer][i][j];
      
      // 跳过没有有效地面数据的节点
      if (!neighbor_node->valid) {
        if (debug_ && closed_set.size() < 10) {  // 只打印前几次
          printf("[DEBUG] Skipping invalid neighbor: layer=%d, row=%d, col=%d, height=%.2f\n",
                 layer, i, j, neighbor_node->height);
        }
        continue;
      }

      if (neighbor_node->cost > cost_threshold_) {
        if (abs(neighbor_node->ele) < 0.5) {
          continue;
        } else {
          // 放宽高度约束：从0.3m增加到2.0m
          // 这允许在不同slice之间更灵活地切换，避免轨迹浮空
          if (std::abs(neighbor_node->height - current_node->height) > 2.0) {
            continue;
          }
        }
      }

      // if ((neighbor_node->cost > cost_threshold_) ||
      //     std::abs(neighbor_node->height - current_node->height) > 0.3) {
      //   continue;
      // }

      auto diff = neighbor_node->idx - current_node->idx;
      double step_cost = step_cost_weight_ * neighbor_node->cost;
      if (step_cost < 5) step_cost = 0.0;
      tentative_g =
          current_node->g +
          std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]) +
          step_cost;

      auto p_neighbor = closed_set.find(GetHash(neighbor_node->idx));
      if (p_neighbor != closed_set.end()) {
        if (tentative_g >= p_neighbor->second->g) {
          continue;
        }
      }

      if (tentative_g < neighbor_node->g) {
        neighbor_node->g = tentative_g;
        neighbor_node->f = tentative_g + GetHeuristic(neighbor_node, goal_node);
        neighbor_node->parent = current_node;
        open_set.push(neighbor_node);
      }
    }
  }

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);
  printf("path not found\n, time elapsed: %f ms\n",
         duration.count() / 1000.0);
  if (debug_) {
    ConvertClosedSetToMatrix(closed_set);
  }
  return false;
}

int Astar::DecideLayer(const Node* cur_node) const {
  int layer = cur_node->layer;
  int i = cur_node->idx[1];
  int j = cur_node->idx[2];
  double cur_height = cur_node->height;

  int true_layer = layer;
  
  // 关键修改：如果当前层在该位置没有有效地面数据（height < -50表示NaN），
  // 必须切换到有有效数据的层
  bool current_layer_invalid = (cur_height < -50.0);
  
  // 改进的层选择策略：找到有效且高度最接近的层
  double min_height_diff = 1e9;
  int best_layer = layer;
  bool found_valid = false;
  
  for (int test_layer = 0; test_layer < max_layers_; ++test_layer) {
    const Node& test_node = grid_map_[test_layer][i][j];
    
    // 跳过无效层
    if (!test_node.valid) {
      continue;
    }
    
    // 跳过成本过高且没有gateway标记的层
    if (test_node.cost > cost_threshold_ && abs(test_node.ele) < 0.5) {
      continue;
    }
    
    double height_diff = std::abs(test_node.height - cur_height);
    
    // 如果当前层无效，必须找到有效层，不考虑高度差限制
    // 如果当前层有效，只在高度差合理时切换
    bool should_update = false;
    if (current_layer_invalid) {
      // 当前层无效，接受任何有效层
      should_update = (height_diff < min_height_diff);
    } else {
      // 当前层有效，只在高度差<2.0m时切换
      should_update = (height_diff < min_height_diff && height_diff < 2.0);
    }
    
    if (should_update) {
      min_height_diff = height_diff;
      best_layer = test_layer;
      found_valid = true;
    }
  }
  
  if (found_valid) {
    true_layer = best_layer;
  }
  
  // 检查是否需要根据ele标记进行层切换（gateway）
  for (const auto offset : search_layers_offset_) {
    int cur_layer = best_layer + offset;

    if (cur_layer < 0 || cur_layer >= max_layers_) {
      continue;
    }

    const Node& search_node = grid_map_[cur_layer][i][j];
    
    // 跳过无效层
    if (!search_node.valid) {
      continue;
    }

    if (abs(search_node.height - cur_height) > 2.0) {  // 放宽到2.0m
      continue;
    }

    if (search_node.ele > 0.5) {
      int next_layer = std::min(cur_layer + 1, max_layers_ - 1);
      // 确保目标层有效
      if (grid_map_[next_layer][i][j].valid) {
        true_layer = next_layer;
      }
      break;
    } else if (search_node.ele < -0.5) {
      int next_layer = std::max(cur_layer - 1, 0);
      // 确保目标层有效
      if (grid_map_[next_layer][i][j].valid) {
        true_layer = next_layer;
      }
      break;
    }
  }

  return true_layer;
}

double Astar::CalculateStepCost(const Node* node1, const Node* node2) const {}

double Astar::GetHeuristic(const Node* node1, const Node* node2) const {
  double cost = 0.0;

  if (h_type_ == kEuclidean) {
    // l2 distance
    cost = (node1->idx - node2->idx).norm();
  } else if (h_type_ == kDiagonal) {
    // octile distance
    Eigen::Vector3i d = node1->idx - node2->idx;
    int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
    int dmin = std::min(dx, std::min(dy, dz));
    int dmax = std::max(dx, std::max(dy, dz));
    int dmid = dx + dy + dz - dmin - dmax;
    double h =
        std::sqrt(3) * dmin + std::sqrt(2) * (dmid - dmin) + (dmax - dmid);
    cost = h;
  } else if (h_type_ == kManhattan) {
    cost = (node1->idx - node2->idx).lpNorm<1>();
  } else {
    assert(false && "not implemented");
  }

  // cost += std::abs(node1->idx[0] - node2->idx[0]) * 10;
  return cost;
}

std::vector<PathPoint> Astar::GetPathPoints() const {
  std::vector<PathPoint> path_points;

  auto size = search_result_.size();
  path_points.resize(size);

  if (size == 0) {
    printf("path is empty\n, convert to path points failed\n");
    return path_points;
  }

  for (size_t i = 0; i < size; ++i) {
    // path_points[i].layer = search_result_[i][0];
    // path_points[i].x = search_result_[i][2];
    // path_points[i].y = search_result_[i][1];
    // if (i > 0) {
    //   path_points[i].heading =
    //       std::atan2(search_result_[i][1] - search_result_[i - 1][1],
    //                  search_result_[i][2] - search_result_[i - 1][2]);
    // }
    path_points[i].layer = search_result_[i]->layer;
    path_points[i].x = search_result_[i]->idx(2);
    path_points[i].y = search_result_[i]->idx(1);
    path_points[i].height = search_result_[i]->height;
    if (i > 0) {
      path_points[i].heading =
          std::atan2(search_result_[i]->idx(1) - search_result_[i - 1]->idx(1),
                     search_result_[i]->idx(2) - search_result_[i - 1]->idx(2));
    }
  }

  if (size > 1) {
    path_points[0].heading = path_points[1].heading;
  }

  return path_points;
}

Eigen::MatrixXd Astar::GetResultMatrix() const {
  if (search_result_.empty()) {
    printf("path is empty\n, convert to matrix failed\n");
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd path_matrix(search_result_.size(), 3);
  for (size_t i = 0; i < search_result_.size(); ++i) {
    path_matrix(i, 0) = search_result_[i]->layer;
    path_matrix(i, 1) = search_result_[i]->idx[1];
    path_matrix(i, 2) = search_result_[i]->idx[2];
  }
  return path_matrix;
}

void Astar::ConvertClosedSetToMatrix(
    const std::unordered_map<int, Node*>& closed_set) {
  visited_set_ = Eigen::MatrixXi(closed_set.size(), 3);
  int count = 0;
  for (auto i = closed_set.begin(); i != closed_set.end(); ++i) {
    visited_set_(count, 0) = i->second->layer;
    visited_set_(count, 1) = i->second->idx[1];
    visited_set_(count, 2) = i->second->idx[2];
    count += 1;
  }
}

std::vector<Eigen::Vector3i> Astar::GetNeighbors(Node* node) const {}

Eigen::MatrixXd Astar::GetCostLayer(int layer) const {
  Eigen::MatrixXd cost_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      cost_layer(i, j) = grid_map_[layer][i][j].cost;
    }
  }
  return cost_layer;
}
Eigen::MatrixXd Astar::GetEleLayer(int layer) const {
  Eigen::MatrixXd ele_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      ele_layer(i, j) = grid_map_[layer][i][j].ele;
    }
  }
  return ele_layer;
}