#include "kdtree.h"

#include <algorithm>

#include "gtest/gtest.h"

struct KDTree2DTest : public ::testing::Test {
  KdTree unit{};
  std::vector<std::vector<float>> points_2d = {
      {-6.2f, 7.0f}, {-6.3f, 8.4f},  {-5.2f, 7.1f}, {-5.7f, 6.3f},
      {7.2f, 6.1f},  {8.0f, 5.3f},   {7.2f, 7.1f},  {0.2f, -7.1f},
      {1.7f, -6.9f}, {-1.2f, -7.2f}, {2.2f, -8.9f}};
};

struct KDTree3DTest : public ::testing::Test {
  KdTree unit{};
  std::vector<std::vector<float>> points_3d_for_unbalanced_tree = {
      {-6.2f, 7.0f, 3.1f},   {-6.3f, 8.4f, 2.8f},  {-5.2f, 7.1f, 4.0f},
      {-5.7f, 6.3f, 3.4f},   {7.2f, 6.1f, -25.8f}, {8.0f, 5.3f, -28.1f},
      {7.2f, 7.1f, -27.3f},  {0.2f, -7.1f, 45.6f}, {1.7f, -6.9f, 43.0f},
      {-1.2f, -7.2f, 44.1f}, {2.2f, -8.9f, 44.8f}};

  // Modified example from http://www.jcgt.org/published/0004/01/03/paper.pdf
  std::vector<std::vector<float>> points_3d_for_balanced_tree = {
      {2.f, 1.f, 5.f},  {8.f, 6.f, 7.f}, {-7.f, 6.f, 8.f}, {-2.f, 4.f, 2.f},
      {-3.f, 7.f, 9.f}, {5.f, 9.f, 1.f}, {9.f, 5.f, 3.f},  {-6.f, 3.f, 5.f},
      {-4.f, 9.f, 5.f}, {4.f, 4.f, 2.f}, {3.f, 7.f, 6.f},  {6.f, 2.f, 8.f},
      {-1.f, 5.f, 1.f}, {1.f, 2.f, 6.f}, {-5.f, 1.f, 3.f}};
};

TEST(KDTreeTest, Instantiation) {
  KdTree unit{};
  EXPECT_TRUE(unit.root == nullptr);
}

/* ************************* Tests with 2D data ************************* */
TEST_F(KDTree2DTest, InsertOneNode) {
  std::vector<float> point{-6.2f, 7.0f};
  int id = 1;
  unit.insert(point, id);
  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    EXPECT_FLOAT_EQ(point.at(0), unit.root->point.at(0));
    EXPECT_FLOAT_EQ(point.at(1), unit.root->point.at(1));
    EXPECT_EQ(id, unit.root->id);
    EXPECT_TRUE(unit.root->left == nullptr);
    EXPECT_TRUE(unit.root->right == nullptr);
  }
}

TEST_F(KDTree2DTest, InsertTwoNodes_Root_Left) {
  // insert the first point which is the root
  std::vector<float> point1{-6.2f, 7.0f};
  int id1 = 1;
  unit.insert(point1, id1);

  // insert the second point branching to the left
  std::vector<float> point2{-6.3f, 8.4f};
  int id2 = 2;
  unit.insert(point2, id2);

  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    EXPECT_TRUE(unit.root->point == point1);
    EXPECT_TRUE(unit.root->id == id1);
    EXPECT_FALSE(unit.root->left == nullptr);
    EXPECT_TRUE(unit.root->right == nullptr);
    if (unit.root->left != nullptr) {
      EXPECT_FLOAT_EQ(point2.at(0), unit.root->left->point.at(0));
      EXPECT_FLOAT_EQ(point2.at(1), unit.root->left->point.at(1));
      EXPECT_EQ(id2, unit.root->left->id);
    }
  }
}

TEST_F(KDTree2DTest, InsertTwoNodes_Root_Right) {
  // insert the first point which is the root
  std::vector<float> point1{-6.2f, 7.0f};
  int id1 = 1;
  unit.insert(point1, id1);

  // insert the second point branching to the right
  std::vector<float> point2{5.2f, 7.1f};
  int id2 = 2;
  unit.insert(point2, id2);

  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    EXPECT_TRUE(unit.root->point == point1);
    EXPECT_TRUE(unit.root->id == id1);
    EXPECT_TRUE(unit.root->left == nullptr);
    EXPECT_FALSE(unit.root->right == nullptr);
    if (unit.root->right != nullptr) {
      EXPECT_FLOAT_EQ(point2.at(0), unit.root->right->point.at(0));
      EXPECT_FLOAT_EQ(point2.at(1), unit.root->right->point.at(1));
      EXPECT_EQ(id2, unit.root->right->id);
    }
  }
}

TEST_F(KDTree2DTest, InsertFiveNodes) {
  // insert the first point which is the root
  std::vector<float> point1{-6.2f, 7.0f};
  int id1 = 1;
  unit.insert(point1, id1);

  // insert the second point branching from root to the left
  std::vector<float> point2{-6.3f, 8.4f};
  int id2 = 2;
  unit.insert(point2, id2);

  // insert the third point branching from root to the right
  std::vector<float> point3{5.2f, 7.1f};
  int id3 = 3;
  unit.insert(point3, id3);

  // insert the fourth point branching from root over the right to the left
  std::vector<float> point4{5.7f, 6.3f};
  int id4 = 4;
  unit.insert(point4, id4);

  // insert the fifth point branching from root over the right then over the
  // left to the right
  std::vector<float> point5{7.2f, 6.1f};
  int id5 = 5;
  unit.insert(point5, id5);

  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    // Level 0 - Root-Level
    EXPECT_FLOAT_EQ(point1.at(0), unit.root->point.at(0));
    EXPECT_FLOAT_EQ(point1.at(1), unit.root->point.at(1));
    EXPECT_EQ(id1, unit.root->id);
    EXPECT_FALSE(unit.root->left == nullptr);
    EXPECT_FALSE(unit.root->right == nullptr);
    if (unit.root->left != nullptr) {
      // Level 1
      EXPECT_FLOAT_EQ(point2.at(0), unit.root->left->point.at(0));
      EXPECT_FLOAT_EQ(point2.at(1), unit.root->left->point.at(1));
      EXPECT_EQ(id2, unit.root->left->id);
      EXPECT_TRUE(unit.root->left->left == nullptr);
      EXPECT_TRUE(unit.root->left->right == nullptr);
    }
    if (unit.root->right != nullptr) {
      // Level 1
      EXPECT_FLOAT_EQ(point3.at(0), unit.root->right->point.at(0));
      EXPECT_FLOAT_EQ(point3.at(1), unit.root->right->point.at(1));
      EXPECT_EQ(id3, unit.root->right->id);
      EXPECT_FALSE(unit.root->right->left == nullptr);
      EXPECT_TRUE(unit.root->right->right == nullptr);
      if (unit.root->right->left != nullptr) {
        // Level 2
        EXPECT_FLOAT_EQ(point4.at(0), unit.root->right->left->point.at(0));
        EXPECT_FLOAT_EQ(point4.at(1), unit.root->right->left->point.at(1));
        EXPECT_EQ(id4, unit.root->right->left->id);
        EXPECT_TRUE(unit.root->right->left->left == nullptr);
        EXPECT_FALSE(unit.root->right->left->right == nullptr);
        if (unit.root->right->left->right != nullptr) {
          // Level 3
          EXPECT_FLOAT_EQ(point5.at(0),
                          unit.root->right->left->right->point.at(0));
          EXPECT_FLOAT_EQ(point5.at(1),
                          unit.root->right->left->right->point.at(1));
          EXPECT_EQ(id5, unit.root->right->left->right->id);
          EXPECT_TRUE(unit.root->right->left->right->left == nullptr);
          EXPECT_TRUE(unit.root->right->left->right->right == nullptr);
        }
      }
    }
  }
}

TEST_F(KDTree2DTest,
       Search_Nearby_Neighbors_UpperLeftCorner_In_Unbalanced_Tree) {
  for (int i = 0; i < points_2d.size(); i++) {
    unit.insert(points_2d[i], i);
  }

  std::vector<int> expected_value{0, 1, 2, 3};
  std::vector<int> actual_value = unit.search({-6, 7}, 3.0);
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTree2DTest,
       Search_Nearby_Neighbors_UpperRightCorner_In_Unbalanced_Tree) {
  for (int i = 0; i < points_2d.size(); i++) {
    unit.insert(points_2d[i], i);
  }

  std::vector<int> expected_value{4, 5, 6};
  std::vector<int> actual_value = unit.search({7.5f, 6.0}, 3.0);
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTree2DTest, Search_Nearby_Neighbors_Down_In_Unbalanced_Tree) {
  for (int i = 0; i < points_2d.size(); i++) {
    unit.insert(points_2d[i], i);
  }

  std::vector<int> expected_value{7, 8, 9, 10};
  std::vector<int> actual_value = unit.search({1.0f, -8.0f}, 3.0);
  std::sort(actual_value.begin(), actual_value.end());
  EXPECT_EQ(expected_value, actual_value);
}

/* ************************* Tests with 3D data ************************* */

TEST_F(KDTree3DTest, InsertOneNode) {
  std::vector<float> point{1.2f, 3.4f, 5.6f};
  int id = 1;
  unit.insert(point, id);
  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    EXPECT_FLOAT_EQ(point.at(0), unit.root->point.at(0));
    EXPECT_FLOAT_EQ(point.at(1), unit.root->point.at(1));
    EXPECT_FLOAT_EQ(point.at(2), unit.root->point.at(2));
    EXPECT_EQ(id, unit.root->id);
    EXPECT_TRUE(unit.root->left == nullptr);
    EXPECT_TRUE(unit.root->right == nullptr);
  }
}

TEST_F(KDTree3DTest, InsertTwoNodes_Root_Left) {
  // insert the first point which is the root
  std::vector<float> point1{-6.2f, 7.0f, 3.1f};
  int id1 = 1;
  unit.insert(point1, id1);

  // insert the second point branching to the left
  std::vector<float> point2{-6.3f, 8.4f, 2.8f};
  int id2 = 2;
  unit.insert(point2, id2);

  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    EXPECT_TRUE(unit.root->point == point1);
    EXPECT_TRUE(unit.root->id == id1);
    EXPECT_FALSE(unit.root->left == nullptr);
    EXPECT_TRUE(unit.root->right == nullptr);
    if (unit.root->left != nullptr) {
      EXPECT_FLOAT_EQ(point2.at(0), unit.root->left->point.at(0));
      EXPECT_FLOAT_EQ(point2.at(1), unit.root->left->point.at(1));
      EXPECT_FLOAT_EQ(point2.at(2), unit.root->left->point.at(2));
      EXPECT_EQ(id2, unit.root->left->id);
    }
  }
}

TEST_F(KDTree3DTest, InsertTwoNodes_Root_Right) {
  // insert the first point which is the root
  std::vector<float> point1{-6.2f, 7.0f, 3.1f};
  int id1 = 1;
  unit.insert(point1, id1);

  // insert the second point branching to the right
  std::vector<float> point2{5.2f, 7.1f, 4.0f};
  int id2 = 2;
  unit.insert(point2, id2);

  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    EXPECT_TRUE(unit.root->point == point1);
    EXPECT_TRUE(unit.root->id == id1);
    EXPECT_TRUE(unit.root->left == nullptr);
    EXPECT_FALSE(unit.root->right == nullptr);
    if (unit.root->right != nullptr) {
      EXPECT_FLOAT_EQ(point2.at(0), unit.root->right->point.at(0));
      EXPECT_FLOAT_EQ(point2.at(1), unit.root->right->point.at(1));
      EXPECT_FLOAT_EQ(point2.at(2), unit.root->right->point.at(2));
      EXPECT_EQ(id2, unit.root->right->id);
    }
  }
}

TEST_F(KDTree3DTest, InsertFiveNodes) {
  // insert the first point which is the root
  std::vector<float> point1{-6.2f, 7.0f, 3.1f};
  int id1 = 1;
  unit.insert(point1, id1);

  // insert the second point branching from root to the left
  std::vector<float> point2{-6.3f, 8.4f, 2.8f};
  int id2 = 2;
  unit.insert(point2, id2);

  // insert the third point branching from root to the right
  std::vector<float> point3{5.2f, 7.1f, 4.0f};
  int id3 = 3;
  unit.insert(point3, id3);

  // insert the fourth point branching from root over the right to the left
  std::vector<float> point4{-5.7f, 6.3f, 3.4};
  int id4 = 4;
  unit.insert(point4, id4);

  // insert the fifth point branching from root over the right then over the
  // left to the left
  std::vector<float> point5{7.2f, 6.1f, -25.8};
  int id5 = 5;
  unit.insert(point5, id5);

  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    // Level 0 - Root-Level
    EXPECT_FLOAT_EQ(point1.at(0), unit.root->point.at(0));
    EXPECT_FLOAT_EQ(point1.at(1), unit.root->point.at(1));
    EXPECT_FLOAT_EQ(point1.at(2), unit.root->point.at(2));
    EXPECT_EQ(id1, unit.root->id);
    EXPECT_FALSE(unit.root->left == nullptr);
    EXPECT_FALSE(unit.root->right == nullptr);
    if (unit.root->left != nullptr) {
      // Level 1
      EXPECT_FLOAT_EQ(point2.at(0), unit.root->left->point.at(0));
      EXPECT_FLOAT_EQ(point2.at(1), unit.root->left->point.at(1));
      EXPECT_FLOAT_EQ(point2.at(2), unit.root->left->point.at(2));
      EXPECT_EQ(id2, unit.root->left->id);
      EXPECT_TRUE(unit.root->left->left == nullptr);
      EXPECT_TRUE(unit.root->left->right == nullptr);
    }
    if (unit.root->right != nullptr) {
      // Level 1
      EXPECT_FLOAT_EQ(point3.at(0), unit.root->right->point.at(0));
      EXPECT_FLOAT_EQ(point3.at(1), unit.root->right->point.at(1));
      EXPECT_FLOAT_EQ(point3.at(2), unit.root->right->point.at(2));
      EXPECT_EQ(id3, unit.root->right->id);
      EXPECT_FALSE(unit.root->right->left == nullptr);
      EXPECT_TRUE(unit.root->right->right == nullptr);
      if (unit.root->right->left != nullptr) {
        // Level 2
        EXPECT_FLOAT_EQ(point4.at(0), unit.root->right->left->point.at(0));
        EXPECT_FLOAT_EQ(point4.at(1), unit.root->right->left->point.at(1));
        EXPECT_FLOAT_EQ(point4.at(2), unit.root->right->left->point.at(2));
        EXPECT_EQ(id4, unit.root->right->left->id);
        EXPECT_FALSE(unit.root->right->left->left == nullptr);
        EXPECT_TRUE(unit.root->right->left->right == nullptr);
        if (unit.root->right->left->left != nullptr) {
          // Level 3
          EXPECT_FLOAT_EQ(point5.at(0),
                          unit.root->right->left->left->point.at(0));
          EXPECT_FLOAT_EQ(point5.at(1),
                          unit.root->right->left->left->point.at(1));
          EXPECT_FLOAT_EQ(point5.at(2),
                          unit.root->right->left->left->point.at(2));
          EXPECT_EQ(id5, unit.root->right->left->left->id);
          EXPECT_TRUE(unit.root->right->left->left->left == nullptr);
          EXPECT_TRUE(unit.root->right->left->left->right == nullptr);
        }
      }
    }
  }
}

TEST_F(KDTree3DTest,
       Search_Nearby_Neighbors_UpperLeftCorner_In_Unbalanced_Tree) {
  for (int i = 0; i < points_3d_for_unbalanced_tree.size(); i++) {
    unit.insert(points_3d_for_unbalanced_tree[i], i);
  }

  std::vector<int> expected_value{0, 1, 2, 3};
  std::vector<int> actual_value = unit.search({-6.0f, 7.0f, 3.0f}, 3.0f);
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTree3DTest, Search_Nearby_Neighbors_UpperLeftCorner_In_Balanced_Tree) {
  unit.CreateBalancedKDTree(points_3d_for_unbalanced_tree, &unit.root);

  std::vector<int> expected_value{0, 1, 2, 3};
  std::vector<int> actual_value = unit.search({-6.0f, 7.0f, 3.0f}, 3.0f);
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTree3DTest,
       Search_Nearby_Neighbors_UpperRightCorner_In_Unbalanced_Tree) {
  for (int i = 0; i < points_3d_for_unbalanced_tree.size(); i++) {
    unit.insert(points_3d_for_unbalanced_tree[i], i);
  }

  std::vector<int> expected_value{4, 5, 6};
  std::vector<int> actual_value = unit.search({7.5f, 6.0f, -26.0f}, 3.0f);
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTree3DTest,
       Search_Nearby_Neighbors_UpperRightCorner_In_Balanced_Tree) {
  unit.CreateBalancedKDTree(points_3d_for_unbalanced_tree, &unit.root);
  std::vector<int> expected_value{4, 5, 6};
  std::vector<int> actual_value = unit.search({7.5f, 6.0f, -26.0f}, 3.0f);
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTree3DTest, Search_Nearby_Neighbors_Down_In_Unbalanced_Tree) {
  for (int i = 0; i < points_3d_for_unbalanced_tree.size(); i++) {
    unit.insert(points_3d_for_unbalanced_tree[i], i);
  }

  std::vector<int> expected_value{7, 8, 9, 10};
  std::vector<int> actual_value = unit.search({1.0f, -8.0f, 43.5f}, 3.0f);
  std::sort(actual_value.begin(), actual_value.end());
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTree3DTest, Search_Nearby_Neighbors_Down_In_Balanced_Tree) {
  unit.CreateBalancedKDTree(points_3d_for_unbalanced_tree, &unit.root);

  std::vector<int> expected_value{7, 8, 9, 10};
  std::vector<int> actual_value = unit.search({1.0f, -8.0f, 43.5f}, 3.0f);
  std::sort(actual_value.begin(), actual_value.end());
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTree3DTest, Create_Empty_Balanced_3D_Tree) {
  std::vector<std::vector<float>> empty_point_vector{};
  unit.CreateBalancedKDTree(empty_point_vector, &unit.root);
  EXPECT_TRUE(unit.root == nullptr);
}

TEST_F(KDTree3DTest, Create_Balanced_3D_Tree_With_One_Point) {
  std::vector<std::vector<float>> points{{1.f, 2.f, 3.f}};
  unit.CreateBalancedKDTree(points, &unit.root);
  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    EXPECT_EQ(0, unit.root->id);
    EXPECT_FLOAT_EQ(points[0][0], unit.root->point.at(0));
    EXPECT_FLOAT_EQ(points[0][1], unit.root->point.at(1));
    EXPECT_FLOAT_EQ(points[0][2], unit.root->point.at(2));
  }
}

TEST_F(KDTree3DTest, Create_Balanced_3D_Tree_With_Two_Points) {
  std::vector<std::vector<float>> points{{1.f, 2.f, 3.f}, {4.f, 5.f, 6.f}};
  unit.CreateBalancedKDTree(points, &unit.root);
  EXPECT_FALSE(unit.root == nullptr);

  if (unit.root != nullptr) {
    EXPECT_FLOAT_EQ(points[0][0], unit.root->point.at(0));
    EXPECT_FLOAT_EQ(points[0][1], unit.root->point.at(1));
    EXPECT_FLOAT_EQ(points[0][2], unit.root->point.at(2));
    EXPECT_EQ(0, unit.root->id);

    EXPECT_TRUE(unit.root->left == nullptr);
    EXPECT_FALSE(unit.root->right == nullptr);
    if (unit.root->right != nullptr) {
      EXPECT_FLOAT_EQ(points[1][0], unit.root->right->point.at(0));
      EXPECT_FLOAT_EQ(points[1][1], unit.root->right->point.at(1));
      EXPECT_FLOAT_EQ(points[1][2], unit.root->right->point.at(2));
      EXPECT_EQ(1, unit.root->right->id);
    }
  }
}

TEST_F(KDTree3DTest, Create_Balanced_3D_Tree_With_Eleven_Points) {
  unit.CreateBalancedKDTree(points_3d_for_unbalanced_tree, &unit.root);

  std::vector<float> level0_root{0.2f, -7.1f, 45.6f};

  std::vector<float> level1_left_root{-6.2f, 7.f, 3.1};
  std::vector<float> level1_right_root{8.f, 5.3f, -28.1};

  std::vector<float> level2_left_left_root{-5.7f, 6.3f, 3.4f};
  std::vector<float> level2_left_right_root{-6.3f, 8.4f, 2.8f};
  std::vector<float> level2_right_left_root{1.7f, -6.9f, 43.f};
  std::vector<float> level2_right_right_root{7.2f, 7.1f, -27.3f};

  std::vector<float> level3_left_left_right_leaf{-1.2f, -7.2f, 44.1f};

  std::vector<float> level3_left_right_right_leaf{-5.2f, 7.1f, 4.f};

  std::vector<float> level3_right_left_right_leaf{2.2f, -8.9f, 44.8f};

  std::vector<float> level3_right_right_right_leaf{7.2f, 6.1f, -25.8f};

  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    // Level 0 - Root-Level
    EXPECT_FLOAT_EQ(level0_root.at(0), unit.root->point.at(0));
    EXPECT_FLOAT_EQ(level0_root.at(1), unit.root->point.at(1));
    EXPECT_FLOAT_EQ(level0_root.at(2), unit.root->point.at(2));
    // EXPECT_EQ(0, unit.root->id);
    EXPECT_FALSE(unit.root->left == nullptr);
    EXPECT_FALSE(unit.root->right == nullptr);
    if (unit.root->left != nullptr) {
      // Level 1 - left branch
      EXPECT_FLOAT_EQ(level1_left_root.at(0), unit.root->left->point.at(0));
      EXPECT_FLOAT_EQ(level1_left_root.at(1), unit.root->left->point.at(1));
      EXPECT_FLOAT_EQ(level1_left_root.at(2), unit.root->left->point.at(2));
      // EXPECT_EQ(1, unit.root->left->id);
      EXPECT_FALSE(unit.root->left->left == nullptr);
      EXPECT_FALSE(unit.root->left->right == nullptr);
      if (unit.root->left->left != nullptr) {
        // Level 2
        EXPECT_FLOAT_EQ(level2_left_left_root.at(0),
                        unit.root->left->left->point.at(0));
        EXPECT_FLOAT_EQ(level2_left_left_root.at(1),
                        unit.root->left->left->point.at(1));
        EXPECT_FLOAT_EQ(level2_left_left_root.at(2),
                        unit.root->left->left->point.at(2));
        // EXPECT_EQ(id4, unit.root->left->left->id);
        EXPECT_TRUE(unit.root->left->left->left == nullptr);
        EXPECT_FALSE(unit.root->left->left->right == nullptr);
        if (unit.root->left->left->right != nullptr) {
          // Level 3
          EXPECT_FLOAT_EQ(level3_left_left_right_leaf.at(0),
                          unit.root->left->left->right->point.at(0));
          EXPECT_FLOAT_EQ(level3_left_left_right_leaf.at(1),
                          unit.root->left->left->right->point.at(1));
          EXPECT_FLOAT_EQ(level3_left_left_right_leaf.at(2),
                          unit.root->left->left->right->point.at(2));
          // EXPECT_EQ(id4, unit.root->left->left->right->id);
          EXPECT_TRUE(unit.root->left->left->right->left == nullptr);
          EXPECT_TRUE(unit.root->left->left->right->right == nullptr);
        }
      }
      if (unit.root->left->right != nullptr) {
        // Level 2
        EXPECT_FLOAT_EQ(level2_left_right_root.at(0),
                        unit.root->left->right->point.at(0));
        EXPECT_FLOAT_EQ(level2_left_right_root.at(1),
                        unit.root->left->right->point.at(1));
        EXPECT_FLOAT_EQ(level2_left_right_root.at(2),
                        unit.root->left->right->point.at(2));
        // EXPECT_EQ(id4, unit.root->left->right->id);
        EXPECT_TRUE(unit.root->left->right->left == nullptr);
        EXPECT_FALSE(unit.root->left->right->right == nullptr);

        if (unit.root->left->right->right != nullptr) {
          // Level 3
          EXPECT_FLOAT_EQ(level3_left_right_right_leaf.at(0),
                          unit.root->left->right->right->point.at(0));
          EXPECT_FLOAT_EQ(level3_left_right_right_leaf.at(1),
                          unit.root->left->right->right->point.at(1));
          EXPECT_FLOAT_EQ(level3_left_right_right_leaf.at(2),
                          unit.root->left->right->right->point.at(2));
          // EXPECT_EQ(id4, unit.root->left->right->left->id);
          EXPECT_TRUE(unit.root->left->right->right->left == nullptr);
          EXPECT_TRUE(unit.root->left->right->right->right == nullptr);
        }
      }
    }
    if (unit.root->right != nullptr) {
      // Level 1  - right branch
      EXPECT_FLOAT_EQ(level1_right_root.at(0), unit.root->right->point.at(0));
      EXPECT_FLOAT_EQ(level1_right_root.at(1), unit.root->right->point.at(1));
      EXPECT_FLOAT_EQ(level1_right_root.at(2), unit.root->right->point.at(2));
      // EXPECT_EQ(2, unit.root->right->id);
      EXPECT_FALSE(unit.root->right->left == nullptr);
      EXPECT_FALSE(unit.root->right->right == nullptr);
      if (unit.root->right->left != nullptr) {
        // Level 2 - right branch
        EXPECT_FLOAT_EQ(level2_right_left_root.at(0),
                        unit.root->right->left->point.at(0));
        EXPECT_FLOAT_EQ(level2_right_left_root.at(1),
                        unit.root->right->left->point.at(1));
        EXPECT_FLOAT_EQ(level2_right_left_root.at(2),
                        unit.root->right->left->point.at(2));
        // EXPECT_EQ(id4, unit.root->right->left->id);
        EXPECT_TRUE(unit.root->right->left->left == nullptr);
        EXPECT_FALSE(unit.root->right->left->right == nullptr);
        if (unit.root->right->left->right != nullptr) {
          // Level 3
          EXPECT_FLOAT_EQ(level3_right_left_right_leaf.at(0),
                          unit.root->right->left->right->point.at(0));
          EXPECT_FLOAT_EQ(level3_right_left_right_leaf.at(1),
                          unit.root->right->left->right->point.at(1));
          EXPECT_FLOAT_EQ(level3_right_left_right_leaf.at(2),
                          unit.root->right->left->right->point.at(2));
          // EXPECT_EQ(id5, unit.root->right->left->left->id);
          EXPECT_TRUE(unit.root->right->left->right->left == nullptr);
          EXPECT_TRUE(unit.root->right->left->right->right == nullptr);
        }
      }
      if (unit.root->right->right != nullptr) {
        // Level 2 - most right branch
        EXPECT_FLOAT_EQ(level2_right_right_root.at(0),
                        unit.root->right->right->point.at(0));
        EXPECT_FLOAT_EQ(level2_right_right_root.at(1),
                        unit.root->right->right->point.at(1));
        EXPECT_FLOAT_EQ(level2_right_right_root.at(2),
                        unit.root->right->right->point.at(2));
        // EXPECT_EQ(id4, unit.root->right->right->id);
        EXPECT_TRUE(unit.root->right->right->left == nullptr);
        EXPECT_FALSE(unit.root->right->right->right == nullptr);
        if (unit.root->right->right->right != nullptr) {
          // Level 3 - most right branch's left leaf
          EXPECT_FLOAT_EQ(level3_right_right_right_leaf.at(0),
                          unit.root->right->right->right->point.at(0));
          EXPECT_FLOAT_EQ(level3_right_right_right_leaf.at(1),
                          unit.root->right->right->right->point.at(1));
          EXPECT_FLOAT_EQ(level3_right_right_right_leaf.at(2),
                          unit.root->right->right->right->point.at(2));
          // EXPECT_EQ(id5, unit.root->right->right->left->id);
          EXPECT_TRUE(unit.root->right->right->right->left == nullptr);
          EXPECT_TRUE(unit.root->right->right->right->right == nullptr);
        }
      }
    }
  }
}

TEST_F(KDTree3DTest, Create_Balanced_3D_Tree_With_Fifteen_Points) {
  unit.CreateBalancedKDTree(points_3d_for_balanced_tree, &unit.root);

  std::vector<float> level0_root{1, 2, 6};

  std::vector<float> level1_left_root{-1, 5, 1};
  std::vector<float> level1_right_root{9, 5, 3};

  std::vector<float> level2_left_left_root{-5, 1, 3};
  std::vector<float> level2_left_right_root{-7, 6, 8};
  std::vector<float> level2_right_left_root{2, 1, 5};
  std::vector<float> level2_right_right_root{3, 7, 6};

  std::vector<float> level3_left_left_left_leaf{-2, 4, 2};
  std::vector<float> level3_left_left_right_leaf{-6, 3, 5};
  std::vector<float> level3_left_right_left_leaf{-4, 9, 5};
  std::vector<float> level3_left_right_right_leaf{-3, 7, 9};
  std::vector<float> level3_right_left_left_leaf{4, 4, 2};
  std::vector<float> level3_right_left_right_leaf{6, 2, 8};
  std::vector<float> level3_right_right_left_leaf{5, 9, 1};
  std::vector<float> level3_right_right_right_leaf{8, 6, 7};

  EXPECT_FALSE(unit.root == nullptr);
  if (unit.root != nullptr) {
    // Level 0 - Root-Level
    EXPECT_FLOAT_EQ(level0_root.at(0), unit.root->point.at(0));
    EXPECT_FLOAT_EQ(level0_root.at(1), unit.root->point.at(1));
    EXPECT_FLOAT_EQ(level0_root.at(2), unit.root->point.at(2));
    // EXPECT_EQ(0, unit.root->id);
    EXPECT_FALSE(unit.root->left == nullptr);
    EXPECT_FALSE(unit.root->right == nullptr);
    if (unit.root->left != nullptr) {
      // Level 1 - left branch
      EXPECT_FLOAT_EQ(level1_left_root.at(0), unit.root->left->point.at(0));
      EXPECT_FLOAT_EQ(level1_left_root.at(1), unit.root->left->point.at(1));
      EXPECT_FLOAT_EQ(level1_left_root.at(2), unit.root->left->point.at(2));
      // EXPECT_EQ(1, unit.root->left->id);
      EXPECT_FALSE(unit.root->left->left == nullptr);
      EXPECT_FALSE(unit.root->left->right == nullptr);
      if (unit.root->left->left != nullptr) {
        // Level 2 - most left branch
        EXPECT_FLOAT_EQ(level2_left_left_root.at(0),
                        unit.root->left->left->point.at(0));
        EXPECT_FLOAT_EQ(level2_left_left_root.at(1),
                        unit.root->left->left->point.at(1));
        EXPECT_FLOAT_EQ(level2_left_left_root.at(2),
                        unit.root->left->left->point.at(2));
        // EXPECT_EQ(id4, unit.root->left->left->id);
        EXPECT_FALSE(unit.root->left->left->left == nullptr);
        EXPECT_FALSE(unit.root->left->left->right == nullptr);
        if (unit.root->left->left->left != nullptr) {
          // Level 3 - most left branch's left leaf
          EXPECT_FLOAT_EQ(level3_left_left_left_leaf.at(0),
                          unit.root->left->left->left->point.at(0));
          EXPECT_FLOAT_EQ(level3_left_left_left_leaf.at(1),
                          unit.root->left->left->left->point.at(1));
          EXPECT_FLOAT_EQ(level3_left_left_left_leaf.at(2),
                          unit.root->left->left->left->point.at(2));
          // EXPECT_EQ(id4, unit.root->left->left->left->id);
          EXPECT_TRUE(unit.root->left->left->left->left == nullptr);
          EXPECT_TRUE(unit.root->left->left->left->right == nullptr);
        }
        if (unit.root->left->left->right != nullptr) {
          // Level 3 - most left branch's right leaf
          EXPECT_FLOAT_EQ(level3_left_left_right_leaf.at(0),
                          unit.root->left->left->right->point.at(0));
          EXPECT_FLOAT_EQ(level3_left_left_right_leaf.at(1),
                          unit.root->left->left->right->point.at(1));
          EXPECT_FLOAT_EQ(level3_left_left_right_leaf.at(2),
                          unit.root->left->left->right->point.at(2));
          // EXPECT_EQ(id45555, unit.root->left->left->right->id);
          EXPECT_TRUE(unit.root->left->left->right->left == nullptr);
          EXPECT_TRUE(unit.root->left->left->right->right == nullptr);
        }
      }
      if (unit.root->left->right != nullptr) {
        // Level 2 - left branch
        EXPECT_FLOAT_EQ(level2_left_right_root.at(0),
                        unit.root->left->right->point.at(0));
        EXPECT_FLOAT_EQ(level2_left_right_root.at(1),
                        unit.root->left->right->point.at(1));
        EXPECT_FLOAT_EQ(level2_left_right_root.at(2),
                        unit.root->left->right->point.at(2));
        // EXPECT_EQ(id4, unit.root->left->right->id);
        EXPECT_FALSE(unit.root->left->right->left == nullptr);
        EXPECT_FALSE(unit.root->left->right->right == nullptr);

        if (unit.root->left->right->left != nullptr) {
          // Level 3 - left branch's left leaf
          EXPECT_FLOAT_EQ(level3_left_right_left_leaf.at(0),
                          unit.root->left->right->left->point.at(0));
          EXPECT_FLOAT_EQ(level3_left_right_left_leaf.at(1),
                          unit.root->left->right->left->point.at(1));
          EXPECT_FLOAT_EQ(level3_left_right_left_leaf.at(2),
                          unit.root->left->right->left->point.at(2));
          // EXPECT_EQ(id4, unit.root->left->right->left->id);
          EXPECT_TRUE(unit.root->left->right->left->left == nullptr);
          EXPECT_TRUE(unit.root->left->right->left->right == nullptr);
        }
        if (unit.root->left->right->right != nullptr) {
          // Level 3 - left branch's right leaf
          EXPECT_FLOAT_EQ(level3_left_right_right_leaf.at(0),
                          unit.root->left->right->right->point.at(0));
          EXPECT_FLOAT_EQ(level3_left_right_right_leaf.at(1),
                          unit.root->left->right->right->point.at(1));
          EXPECT_FLOAT_EQ(level3_left_right_right_leaf.at(2),
                          unit.root->left->right->right->point.at(2));
          // EXPECT_EQ(id45555, unit.root->left->right->right->id);
          EXPECT_TRUE(unit.root->left->right->right->left == nullptr);
          EXPECT_TRUE(unit.root->left->right->right->right == nullptr);
        }
      }
    }
    if (unit.root->right != nullptr) {
      // Level 1  - right branch
      EXPECT_FLOAT_EQ(level1_right_root.at(0), unit.root->right->point.at(0));
      EXPECT_FLOAT_EQ(level1_right_root.at(1), unit.root->right->point.at(1));
      EXPECT_FLOAT_EQ(level1_right_root.at(2), unit.root->right->point.at(2));
      // EXPECT_EQ(2, unit.root->right->id);
      EXPECT_FALSE(unit.root->right->left == nullptr);
      EXPECT_FALSE(unit.root->right->right == nullptr);
      if (unit.root->right->left != nullptr) {
        // Level 2 - right branch
        EXPECT_FLOAT_EQ(level2_right_left_root.at(0),
                        unit.root->right->left->point.at(0));
        EXPECT_FLOAT_EQ(level2_right_left_root.at(1),
                        unit.root->right->left->point.at(1));
        EXPECT_FLOAT_EQ(level2_right_left_root.at(2),
                        unit.root->right->left->point.at(2));
        // EXPECT_EQ(id4, unit.root->right->left->id);
        EXPECT_FALSE(unit.root->right->left->left == nullptr);
        EXPECT_FALSE(unit.root->right->left->right == nullptr);
        if (unit.root->right->left->left != nullptr) {
          // Level 3 - right branch's left leaf
          EXPECT_FLOAT_EQ(level3_right_left_left_leaf.at(0),
                          unit.root->right->left->left->point.at(0));
          EXPECT_FLOAT_EQ(level3_right_left_left_leaf.at(1),
                          unit.root->right->left->left->point.at(1));
          EXPECT_FLOAT_EQ(level3_right_left_left_leaf.at(2),
                          unit.root->right->left->left->point.at(2));
          // EXPECT_EQ(id5, unit.root->right->left->left->id);
          EXPECT_TRUE(unit.root->right->left->left->left == nullptr);
          EXPECT_TRUE(unit.root->right->left->left->right == nullptr);
        }
        if (unit.root->right->left->right != nullptr) {
          // Level 3 - right branch's right leaf
          EXPECT_FLOAT_EQ(level3_right_left_right_leaf.at(0),
                          unit.root->right->left->right->point.at(0));
          EXPECT_FLOAT_EQ(level3_right_left_right_leaf.at(1),
                          unit.root->right->left->right->point.at(1));
          EXPECT_FLOAT_EQ(level3_right_left_right_leaf.at(2),
                          unit.root->right->left->right->point.at(2));
          // EXPECT_EQ(id5, unit.root->right->left->right->id);
          EXPECT_TRUE(unit.root->right->left->right->left == nullptr);
          EXPECT_TRUE(unit.root->right->left->right->right == nullptr);
        }
      }
      if (unit.root->right->right != nullptr) {
        // Level 2 - most right branch
        EXPECT_FLOAT_EQ(level2_right_right_root.at(0),
                        unit.root->right->right->point.at(0));
        EXPECT_FLOAT_EQ(level2_right_right_root.at(1),
                        unit.root->right->right->point.at(1));
        EXPECT_FLOAT_EQ(level2_right_right_root.at(2),
                        unit.root->right->right->point.at(2));
        // EXPECT_EQ(id4, unit.root->right->right->id);
        EXPECT_FALSE(unit.root->right->right->left == nullptr);
        EXPECT_FALSE(unit.root->right->right->right == nullptr);
        if (unit.root->right->right->left != nullptr) {
          // Level 3 - most right branch's left leaf
          EXPECT_FLOAT_EQ(level3_right_right_left_leaf.at(0),
                          unit.root->right->right->left->point.at(0));
          EXPECT_FLOAT_EQ(level3_right_right_left_leaf.at(1),
                          unit.root->right->right->left->point.at(1));
          EXPECT_FLOAT_EQ(level3_right_right_left_leaf.at(2),
                          unit.root->right->right->left->point.at(2));
          // EXPECT_EQ(id5, unit.root->right->right->left->id);
          EXPECT_TRUE(unit.root->right->right->left->left == nullptr);
          EXPECT_TRUE(unit.root->right->right->left->right == nullptr);
        }
        if (unit.root->right->right->right != nullptr) {
          // Level 3 - most right branch's right leaf
          EXPECT_FLOAT_EQ(level3_right_right_right_leaf.at(0),
                          unit.root->right->right->right->point.at(0));
          EXPECT_FLOAT_EQ(level3_right_right_right_leaf.at(1),
                          unit.root->right->right->right->point.at(1));
          EXPECT_FLOAT_EQ(level3_right_right_right_leaf.at(2),
                          unit.root->right->right->right->point.at(2));
          // EXPECT_EQ(id5, unit.root->right->right->right->id);
          EXPECT_TRUE(unit.root->right->right->right->left == nullptr);
          EXPECT_TRUE(unit.root->right->right->right->right == nullptr);
        }
      }
    }
  }
}