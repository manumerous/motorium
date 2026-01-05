#include <gtest/gtest.h>
#include <numeric>

#include <optional>
#include "motorium_model/FixedIDArray.h"

using namespace motorium::model;

namespace {

// Helper struct for testing non-trivial types
struct TestStruct {
  double value = 0.0;
  bool operator==(const TestStruct& other) const { return value == other.value; }
};

}  // namespace

class FixedIDArrayTest : public ::testing::Test {};

TEST_F(FixedIDArrayTest, InitializationCalculatesSizeCorrectly) {
  FixedIDArray<int> array(10);
  EXPECT_EQ(array.size(), 10);
}

TEST_F(FixedIDArrayTest, InitializationFailsForZeroSize) {
  EXPECT_DEATH(FixedIDArray<int> array(0), "Cannot initialize empty map");
}

TEST_F(FixedIDArrayTest, InitializationFailsForTooLargeSize) {
  EXPECT_DEATH(FixedIDArray<int> array(300), "Too many elements");
}

TEST_F(FixedIDArrayTest, AccessWorksForValidIndices) {
  FixedIDArray<int> array(5);
  array[0] = 42;
  array.at(4) = 100;

  EXPECT_EQ(array[0], 42);
  EXPECT_EQ(array.at(4), 100);
}

TEST_F(FixedIDArrayTest, AccessFailsForOutOfBounds) {
  FixedIDArray<int> array(5);
  EXPECT_DEATH(array.at(5), "Element with ID 5 not found");
  EXPECT_DEATH(array[10], "Element with ID 10 not found");
}

TEST_F(FixedIDArrayTest, ConstAccessWorks) {
  FixedIDArray<int> array(5);
  array[2] = 7;

  const FixedIDArray<int>& const_array = array;
  EXPECT_EQ(const_array[2], 7);
  EXPECT_EQ(const_array.at(2), 7);
}

TEST_F(FixedIDArrayTest, AssignmentWorksIdeally) {
  FixedIDArray<int> arr1(3);
  arr1[0] = 1;
  arr1[1] = 2;
  arr1[2] = 3;

  FixedIDArray<int> arr2(3);
  arr2 = arr1;

  EXPECT_EQ(arr2[0], 1);
  EXPECT_EQ(arr2[1], 2);
  EXPECT_EQ(arr2[2], 3);
}

TEST_F(FixedIDArrayTest, AssignmentFailsForSizeMismatch) {
  FixedIDArray<int> arr1(3);
  FixedIDArray<int> arr2(4);
  EXPECT_DEATH(arr2 = arr1, "size mismatch");
}

TEST_F(FixedIDArrayTest, IterationWorks) {
  FixedIDArray<int> array(3);
  array[0] = 10;
  array[1] = 20;
  array[2] = 30;

  int sum = std::accumulate(array.begin(), array.end(), 0);
  EXPECT_EQ(sum, 60);
}

TEST_F(FixedIDArrayTest, ToEigenVectorWorksForSimpleType) {
  FixedIDArray<double> array(3);
  array[0] = 1.1;
  array[1] = 2.2;
  array[2] = 3.3;

  std::vector<size_t> indices = {0, 2};
  auto extractor = [](double v) { return v; };

  auto vec = array.toEigenVector<std::vector<size_t>, double>(indices, extractor);

  ASSERT_EQ(vec.size(), 2);
  EXPECT_DOUBLE_EQ(vec(0), 1.1);
  EXPECT_DOUBLE_EQ(vec(1), 3.3);
}

TEST_F(FixedIDArrayTest, ToEigenVectorWorksWithOptional) {
  FixedIDArray<std::optional<double>> array(3);
  array[0] = 1.0;
  array[1] = std::nullopt;
  array[2] = 3.0;

  std::vector<size_t> indices = {0, 1, 2};
  auto extractor = [](double v) { return v; };

  auto vec = array.toEigenVector<std::vector<size_t>, double>(indices, extractor, -1.0);  // Default -1.0 for nullopt

  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec(0), 1.0);
  EXPECT_DOUBLE_EQ(vec(1), -1.0);
  EXPECT_DOUBLE_EQ(vec(2), 3.0);
}

TEST_F(FixedIDArrayTest, ToEigenVectorWorksWithStruct) {
  FixedIDArray<TestStruct> array(2);
  array[0].value = 5.5;
  array[1].value = 6.6;

  std::vector<size_t> indices = {1};
  auto extractor = [](const TestStruct& s) { return s.value; };

  auto vec = array.toEigenVector<std::vector<size_t>, double>(indices, extractor);

  ASSERT_EQ(vec.size(), 1);
  EXPECT_DOUBLE_EQ(vec(0), 6.6);
}

TEST_F(FixedIDArrayTest, ToEigenVectorWorksWithStructAndOptional) {
  FixedIDArray<std::optional<TestStruct>> array(2);
  array[0] = TestStruct{5.5};
  array[1] = std::nullopt;

  std::vector<size_t> indices = {0, 1};
  auto extractor = [](const TestStruct& s) { return s.value; };

  auto vec = array.toEigenVector<std::vector<size_t>, double>(indices, extractor, -1.0);  // Default -1.0 for nullopt

  ASSERT_EQ(vec.size(), 2);
  EXPECT_DOUBLE_EQ(vec(0), 5.5);
  EXPECT_DOUBLE_EQ(vec(1), -1.0);
}
