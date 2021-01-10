/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KD_TREE_H_
#define KD_TREE_H_

#include <utility>

#include "../../render/render.h"

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
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
    Node **current_node = &root;
    int tree_depth{};
    while (*current_node != nullptr) {
      int lookup_index = tree_depth % 2;
      if (point.at(lookup_index) < (*current_node)->point.at(lookup_index)) {
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
    search_helper(root, target, 0, distanceTol, ids);
    std::sort(ids.begin(), ids.end());
    return ids;
  }

  void search_helper(Node *node, std::vector<float> target, int tree_depth,
                     float dist_tol, std::vector<int> &ids) {
    if (node != nullptr) {
      float x_distance{fabs(node->point.at(0) - target.at(0))};
      float y_distance{fabs(node->point.at(1) - target.at(1))};
      if ((x_distance <= dist_tol) && (y_distance <= dist_tol)) {
        float radius{sqrt(x_distance * x_distance + y_distance * y_distance)};
        if (radius <= dist_tol) {
          ids.push_back(node->id);
        }
      }

      int lookup_index = tree_depth % 2;
      if ((target[lookup_index] - dist_tol) <= node->point.at(lookup_index)) {
        search_helper(node->left, target, tree_depth + 1, dist_tol, ids);
      }
      if ((target[lookup_index] + dist_tol) > node->point.at(lookup_index)) {
        search_helper(node->right, target, tree_depth + 1, dist_tol, ids);
      }
    }
  }
};

#endif  // KD_TREE_H_
