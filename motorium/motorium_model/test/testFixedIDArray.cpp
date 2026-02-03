/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>
#include <numeric>

#include "motorium_core/Types.h"
#include "motorium_model/FixedIDArray.h"

using namespace motorium::model;

namespace {

// Helper struct for testing non-trivial types
struct TestStruct {
  double xyz = 0.0;
  bool operator==(const TestStruct& other) const { return xyz == other.xyz; }
};

// Helper class with getter/setter for testing
class TestClass {
 public:
  TestClass() : data_(0.0) {}
  explicit TestClass(double val) : data_(val) {}

  double getData() const { return data_; }
  void setData(double val) { data_ = val; }

  bool operator==(const TestClass& other) const { return data_ == other.data_; }

 private:
  double data_;
};

}  // namespace

class FixedIDArrayTest : public ::testing::Test {};

TEST_F(FixedIDArrayTest, InitializationCalculatesSizeCorrectly) {
  FixedIDArray<int> array(10);
  EXPECT_EQ(array.size(), 10u);
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
  array[0].xyz = 5.5;
  array[1].xyz = 6.6;

  std::vector<size_t> indices = {1};
  auto extractor = [](const TestStruct& s) { return s.xyz; };

  auto vec = array.toEigenVector<std::vector<size_t>, double>(indices, extractor);

  ASSERT_EQ(vec.size(), 1);
  EXPECT_DOUBLE_EQ(vec(0), 6.6);
}

TEST_F(FixedIDArrayTest, ToEigenVectorWorksWithStructAndOptional) {
  FixedIDArray<std::optional<TestStruct>> array(2);
  array[0] = TestStruct{5.5};
  array[1] = std::nullopt;

  std::vector<size_t> indices = {0, 1};
  auto extractor = [](const TestStruct& s) { return s.xyz; };

  auto vec = array.toEigenVector<std::vector<size_t>, double>(indices, extractor, -1.0);  // Default -1.0 for nullopt

  ASSERT_EQ(vec.size(), 2);
  EXPECT_DOUBLE_EQ(vec(0), 5.5);
  EXPECT_DOUBLE_EQ(vec(1), -1.0);
}

// ============================================================================
// fromEigenVector Tests
// ============================================================================

TEST_F(FixedIDArrayTest, FromEigenVectorWorksForScalar) {
  FixedIDArray<double> array(3);
  array[0] = 0.0;
  array[1] = 0.0;
  array[2] = 0.0;

  std::vector<size_t> indices = {0, 2};
  motorium::vector_t vec(2);
  vec << 1.5, 3.7;

  auto inserter = [](double& target, double value) { target = value; };
  array.fromEigenVector<std::vector<size_t>, double>(indices, vec, inserter);

  EXPECT_DOUBLE_EQ(array[0], 1.5);
  EXPECT_DOUBLE_EQ(array[1], 0.0);  // Unchanged
  EXPECT_DOUBLE_EQ(array[2], 3.7);
}

TEST_F(FixedIDArrayTest, FromEigenVectorWorksForStruct) {
  FixedIDArray<TestStruct> array(3);
  array[0].xyz = 0.0;
  array[1].xyz = 0.0;
  array[2].xyz = 0.0;

  std::vector<size_t> indices = {1, 2};
  motorium::vector_t vec(2);
  vec << 2.5, 4.8;

  auto inserter = [](TestStruct& target, double value) { target.xyz = value; };
  array.fromEigenVector<std::vector<size_t>, double>(indices, vec, inserter);

  EXPECT_DOUBLE_EQ(array[0].xyz, 0.0);  // Unchanged
  EXPECT_DOUBLE_EQ(array[1].xyz, 2.5);
  EXPECT_DOUBLE_EQ(array[2].xyz, 4.8);
}

TEST_F(FixedIDArrayTest, FromEigenVectorWorksForClassWithSetterGetter) {
  FixedIDArray<TestClass> array(3);
  array[0].setData(0.0);
  array[1].setData(0.0);
  array[2].setData(0.0);

  std::vector<size_t> indices = {0, 1, 2};
  motorium::vector_t vec(3);
  vec << 10.5, 20.3, 30.7;

  auto inserter = [](TestClass& target, double value) { target.setData(value); };
  array.fromEigenVector<std::vector<size_t>, double>(indices, vec, inserter);

  EXPECT_DOUBLE_EQ(array[0].getData(), 10.5);
  EXPECT_DOUBLE_EQ(array[1].getData(), 20.3);
  EXPECT_DOUBLE_EQ(array[2].getData(), 30.7);
}

TEST_F(FixedIDArrayTest, FromEigenVectorWorksForOptionalStruct) {
  FixedIDArray<std::optional<TestStruct>> array(5);
  array[0] = TestStruct{1.0};  // Initialized
  array[1] = TestStruct{2.0};  // Initialized
  array[2] = TestStruct{3.0};  // Initialized
  array[3] = std::nullopt;
  array[4] = std::nullopt;

  std::vector<size_t> indices = {0, 2};
  motorium::vector_t vec(2);
  vec << 99.5, 88.3;

  auto inserter = [](TestStruct& target, double value) { target.xyz = value; };
  array.fromEigenVector<std::vector<size_t>, double>(indices, vec, inserter);

  ASSERT_TRUE(array[0].has_value());
  EXPECT_DOUBLE_EQ(array[0]->xyz, 99.5);

  ASSERT_TRUE(array[1].has_value());
  EXPECT_DOUBLE_EQ(array[1]->xyz, 2.0);  // Unchanged

  ASSERT_TRUE(array[2].has_value());
  EXPECT_DOUBLE_EQ(array[2]->xyz, 88.3);
}

TEST_F(FixedIDArrayTest, FromEigenVectorFailsForNulloptStruct) {
  FixedIDArray<std::optional<TestStruct>> array(5);
  array[0] = TestStruct{1.0};  // Initialized
  array[1] = TestStruct{2.0};  // Initialized
  array[2] = TestStruct{3.0};  // Initialized
  array[3] = std::nullopt;
  array[4] = std::nullopt;

  std::vector<size_t> indices = {0, 4};
  motorium::vector_t vec(2);
  vec << 99.5, 88.3;

  auto inserter = [](TestStruct& target, double value) { target.xyz = value; };
  EXPECT_THROW((array.fromEigenVector<std::vector<size_t>, double>(indices, vec, inserter)), std::bad_optional_access);
}

TEST_F(FixedIDArrayTest, ToEigenVectorAndFromEigenVectorRoundTrip) {
  FixedIDArray<TestStruct> array(3);
  array[0].xyz = 1.1;
  array[1].xyz = 2.2;
  array[2].xyz = 3.3;

  std::vector<size_t> indices = {0, 1, 2};

  // Extract to vector
  auto extractor = [](const TestStruct& s) { return s.xyz; };
  auto vec = array.toEigenVector<std::vector<size_t>, double>(indices, extractor);

  // Modify vector
  vec *= 2.0;

  // Write back to array
  auto inserter = [](TestStruct& target, double value) { target.xyz = value; };
  array.fromEigenVector<std::vector<size_t>, double>(indices, vec, inserter);

  EXPECT_DOUBLE_EQ(array[0].xyz, 2.2);
  EXPECT_DOUBLE_EQ(array[1].xyz, 4.4);
  EXPECT_DOUBLE_EQ(array[2].xyz, 6.6);

  // Second test: Apply a random matrix transformation
  // Create a random 3x3 matrix
  motorium::matrix3_t transform = motorium::matrix3_t::Random();

  // Extract current values
  vec = array.toEigenVector<std::vector<size_t>, double>(indices, extractor);

  // Apply transformation
  motorium::vector3_t transformed = transform * vec;

  // Write back transformed values
  array.fromEigenVector<std::vector<size_t>, double>(indices, transformed, inserter);

  EXPECT_NEAR(array[0].xyz, transformed(0), 1e-10);
  EXPECT_NEAR(array[1].xyz, transformed(1), 1e-10);
  EXPECT_NEAR(array[2].xyz, transformed(2), 1e-10);
}
