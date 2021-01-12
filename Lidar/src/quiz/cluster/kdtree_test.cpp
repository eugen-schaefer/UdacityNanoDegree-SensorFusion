#include "kdtree.h"

#include "gtest/gtest.h"

struct KDTreeTest : public ::testing::Test {
  KdTree unit{};
  std::vector<std::vector<float>> points = {
      {-6.2, 7},   {-6.3, 8.4},  {-5.2, 7.1}, {-5.7, 6.3},
      {7.2, 6.1},  {8.0, 5.3},   {7.2, 7.1},  {0.2, -7.1},
      {1.7, -6.9}, {-1.2, -7.2}, {2.2, -8.9}};
};

TEST_F(KDTreeTest, Instantiation) { EXPECT_TRUE(unit.root == nullptr); }

TEST_F(KDTreeTest, InsertOneNode) {
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

TEST_F(KDTreeTest, InsertTwoNodes_Root_Left) {
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

TEST_F(KDTreeTest, InsertTwoNodes_Root_Right) {
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

TEST_F(KDTreeTest, InsertFiveNodes_Root_Left_Root_Right_Left_Right) {
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

TEST_F(KDTreeTest, Search_Nearby_Neighbors_UpperLeftCorner) {
  for (int i = 0; i < points.size(); i++) {
    unit.insert(points[i], i);
  }

  std::vector<int> expected_value{0, 1, 2, 3};
  std::vector<int> actual_value = unit.search({-6, 7}, 3.0);
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTreeTest, Search_Nearby_Neighbors_UpperRightCorner) {
  for (int i = 0; i < points.size(); i++) {
    unit.insert(points[i], i);
  }

  std::vector<int> expected_value{4, 5, 6};
  std::vector<int> actual_value = unit.search({7.5f, 6.0}, 3.0);
  EXPECT_EQ(expected_value, actual_value);
}

TEST_F(KDTreeTest, Search_Nearby_Neighbors_Down) {
  for (int i = 0; i < points.size(); i++) {
    unit.insert(points[i], i);
  }

  std::vector<int> expected_value{7, 8, 9, 10};
  std::vector<int> actual_value = unit.search({1.0f, -8.0f}, 3.0);
  std::sort(actual_value.begin(), actual_value.end());
  EXPECT_EQ(expected_value, actual_value);
}