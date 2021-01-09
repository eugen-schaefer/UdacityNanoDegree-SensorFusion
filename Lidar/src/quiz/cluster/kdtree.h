/* \author Aaron Brown */
// Quiz on implementing kd tree

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
        // the function should create a new node and place correctly with in the root
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
        std::vector<int> ids;
        return ids;
    }
};
