#ifndef KD_TREE_H_
#define KD_TREE_H_

#include <algorithm>
#include <cmath>
#include <vector>

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(std::move(arr)), id(setId), left(nullptr), right(nullptr) {}
};

struct KdTree {
  Node *root;

  KdTree() : root(nullptr) {}

  void insert(std::vector<float> point, int id) {
    // the function should create a new node and place correctly with in the
    // root
    Node **current_node = &root;
    int tree_depth{};
    int dimension{static_cast<int>(point.size())};
    while (*current_node != nullptr) {
      int lookup_index = tree_depth % dimension;
      if (point[lookup_index] < (*current_node)->point[lookup_index]) {
        current_node = &((*current_node)->left);
      } else {
        current_node = &((*current_node)->right);
      }
      tree_depth++;
    }
    *current_node = new Node(point, id);
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids{};
    search_helper(root, std::move(target), 0, distanceTol, ids);
    return ids;
  }

  void search_helper(Node *node, std::vector<float> target, int tree_depth,
                     float dist_tol, std::vector<int> &ids) {
    if (node != nullptr) {
      int dimension{static_cast<int>(target.size())};
      if (dimension == 2) {
        float x_distance{std::fabs(node->point[0] - target[0])};
        float y_distance{std::fabs(node->point[1] - target[1])};
        if ((x_distance <= dist_tol) && (y_distance <= dist_tol)) {
          float radius{
              std::sqrt(x_distance * x_distance + y_distance * y_distance)};
          if (radius <= dist_tol) {
            ids.push_back(node->id);
          }
        }
      } else if (dimension == 3) {
        float x_distance{std::fabs(node->point[0] - target[0])};
        float y_distance{std::fabs(node->point[1] - target[1])};
        float z_distance{std::fabs(node->point[2] - target[2])};
        if ((x_distance <= dist_tol) && (y_distance <= dist_tol) &&
            (z_distance <= dist_tol)) {
          float radius{std::sqrt(x_distance * x_distance +
                                 y_distance * y_distance +
                                 z_distance * z_distance)};
          if (radius <= dist_tol) {
            ids.push_back(node->id);
          }
        }
      }

      int lookup_index = tree_depth % dimension;
      if ((target[lookup_index] - dist_tol) <= node->point.at(lookup_index)) {
        search_helper(node->left, target, tree_depth + 1, dist_tol, ids);
      }
      if ((target[lookup_index] + dist_tol) > node->point.at(lookup_index)) {
        search_helper(node->right, target, tree_depth + 1, dist_tol, ids);
      }
    }
  }

  void CreateBalancedKDTree(std::vector<std::vector<float>> points, Node **mount_node,
              int tree_depth = 0) {
    static int new_id{0};
    if (points.size() < 2) {
      if (points.size() == 1) {
        *mount_node = new Node(points[0], new_id++);
      }
      return;
    }

    int dimension{static_cast<int>(points[0].size())};

    std::sort(
        points.begin(), points.end(),
        [tree_depth, dimension](std::vector<float> a, std::vector<float> b) {
          return (a[tree_depth % dimension] < b[tree_depth % dimension]);
        });
    unsigned int median_index{
        (static_cast<unsigned int>(points.size()) + 1) / 2 - 1};
    *mount_node = new Node(points[median_index], new_id++);

    std::vector<std::vector<float>> left_branch{};
    std::copy(points.begin(), points.begin() + median_index,
              back_inserter(left_branch));
    CreateBalancedKDTree(left_branch, &(*mount_node)->left, tree_depth + 1);

    std::vector<std::vector<float>> right_branch{};
    std::copy(points.end() - median_index, points.end(),
              back_inserter(right_branch));
    CreateBalancedKDTree(right_branch, &(*mount_node)->right, tree_depth + 1);
  }
};

#endif  // KD_TREE_H_
