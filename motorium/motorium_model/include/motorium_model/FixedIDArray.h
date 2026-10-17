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

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <concepts>
#include <memory>
#include <span>

#include <motorium_core/Check.h>

namespace motorium::model {

// The FixedIDArray is a lightweight data container indexed by dense, zero-based
// IDs. By preventing operations leading to dynamic memory allocating and
// providing safe, bounds-checked access, standard iteration, and cache-friendly
// contiguous storage and extraction of per-ID data into Eigen vectors it is
// optimized for realtime programming.

// Helper trait to detect std::optional
template <typename T>
struct is_optional : std::false_type {};
template <typename T>
struct is_optional<std::optional<T>> : std::true_type {};
template <typename T>
inline constexpr bool is_optional_v = is_optional<T>::value;

// Define an extractor that gets a scalar value out of T for eigen vectorization
// when T is itself a composite type (e.g. struct).
template <typename E, typename T, typename ScalarType>
concept IDMapExtractor =
    (requires(const T& val) { val.has_value(); *val; } &&
     requires(E e, const T& val) { { e(*val) } -> std::convertible_to<ScalarType>; }) ||
    (!requires(const T& val) { val.has_value(); } &&
     requires(E e, const T& val) { { e(val) } -> std::convertible_to<ScalarType>; });

// Define an inserter that sets a scalar value into T for eigen vectorization
// when T is itself a composite type (e.g. struct).
template <typename I, typename T, typename ScalarType>
concept IDMapInserter =
    (requires(const T& val) { val.has_value(); *val; } &&
     requires(I i, T& val, ScalarType s) { i(*val, s); }) ||
    (!requires(const T& val) { val.has_value(); } &&
     requires(I i, T& val, ScalarType s) { i(val, s); });

template <typename T>
class FixedIDArray {
 public:
  explicit FixedIDArray(size_t size)
      : map_elements_(new T[size]), map_view_(map_elements_.get(), size), const_map_view_(map_elements_.get(), size) {
    // Limit map size
    MT_CHECK(size <= 255) << "Too many elements in FixedIDArray. max supported size is 255";

    MT_CHECK(size != 0) << "Cannot initialize empty FixedIDArray!";
  }

  // Construct from an existing contiguous container (std::vector, std::array, C
  // array, ...), inferring the size and copying each element as the initial
  // value. The input maps positionally: init[i] becomes the value for ID i.
  // The same constraints as the size constructor apply, but now depend on the
  // input at runtime: init must hold between 1 and 255 elements.
  explicit FixedIDArray(std::span<const T> init) : FixedIDArray(init.size()) { std::copy(init.begin(), init.end(), map_view_.begin()); }
  FixedIDArray() = delete;

  // Deleted to prevent dynamic memory allocation outside construction, as well as errors from naive
  // span copies.
  FixedIDArray(const FixedIDArray<T>& other) = delete;
  FixedIDArray(FixedIDArray<T>&& other) = delete;
  FixedIDArray& operator=(FixedIDArray<T>&& other) = delete;  // Disabled for now. Needs extra care with spans.
  FixedIDArray& operator=(const FixedIDArray<T>& other) {
    // "same type" is enforced by the signature (FixedIDArray<T>).
    // Still check length to ensure maps are compatible.
    MT_CHECK(this->size() == other.size()) << std::format("FixedIDArray::assign size mismatch: this.size()={}, other.size()={}",
                                                          this->size(), other.size());

    const size_t n = this->size();
    for (size_t i = 0; i < n; ++i) {
      map_elements_[i] = other.map_elements_[i];
    }
    return *this;
  }

  bool contains(size_t id) const noexcept { return id < map_view_.size(); }

  size_t size() const { return map_view_.size(); }

  // Bounds are checked with MT_DCHECK, so an out-of-range id is a loud abort in debug and test
  // builds and undefined behaviour in optimized ones. That is deliberate: absl's CHECK is
  // unconditional, so the failure path (constructing the fatal log message and formatting it) sits
  // permanently inside at(), making the body too large for clang to inline. Since at() backs
  // operator[] and every per-joint read in the control loop, that one non-inlinable call forces
  // live values onto the stack every iteration. Under NDEBUG the DCHECK erases that code and the
  // accessor collapses into a register-level array index.
  //
  // The contract this shifts onto callers: validate the id before indexing when it isn't already
  // known good. Indices from RobotDescription lookups are checked at construction.
  T& at(size_t element_id) {
    MT_DCHECK(this->contains(element_id)) << std::format(
        "Element with ID {} not found in IDMap (Max ID for this map is: "
        "{})",
        element_id, map_view_.size() - 1);

    return map_elements_[element_id];
  }

  const T& at(size_t element_id) const {
    MT_DCHECK(this->contains(element_id)) << std::format(
        "Element with ID {} not found in IDMap (Max ID for this map is: "
        "{})",
        element_id, map_view_.size() - 1);

    return map_elements_[element_id];
  }

  T& operator[](size_t element_id) { return at(element_id); }

  const T& operator[](size_t element_id) const { return at(element_id); }

  // Extract one scalar per ID (in `range` order) into a caller-owned Eigen vector. The output must
  // already be sized to the range length; nothing is allocated, so this is safe on the real-time
  // path. The size contract is verified with MT_DCHECK, which compiles out in optimized builds.
  template <typename Range, typename ScalarType>
  void fillEigenVector(const Range& range,
                       Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& vector,
                       IDMapExtractor<T, ScalarType> auto extractor,
                       ScalarType default_value = ScalarType{}) const {
    MT_DCHECK(static_cast<size_t>(vector.size()) == std::ranges::size(range))
        << "fillEigenVector: output vector must be pre-sized to the range length";
    size_t index = 0;

    if constexpr (is_optional_v<T>) {
      for (const auto& id : range) {
        const T& val = this->at(id);
        if (val.has_value()) {
          vector(index) = extractor(*val);
        } else {
          vector(index) = default_value;
        }

        ++index;
      }
    } else {
      for (const auto& id : range) {
        const T& val = this->at(id);

        vector(index) = extractor(val);
        ++index;
      }
    }
  }

  template <typename Range, typename ScalarType>
  void fromEigenVector(const Range& range,
                       const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& vector,
                       IDMapInserter<T, ScalarType> auto inserter) {
    MT_DCHECK(std::ranges::size(range) == static_cast<size_t>(vector.size()));
    size_t index = 0;

    for (const auto& id : range) {
      T& val = this->at(id);

      if constexpr (is_optional_v<T>) {
        if (!val.has_value()) {
          val = typename T::value_type{};  // Initialize if nullopt
        }
        inserter(*val, vector(index));
      } else {
        inserter(val, vector(index));
      }
      ++index;
    }
  }

  // Expose span's iterators as your own
  using iterator = typename std::span<T>::iterator;
  using const_iterator = typename std::span<const T>::iterator;
  using reverse_iterator = typename std::span<T>::reverse_iterator;
  using const_reverse_iterator = typename std::span<const T>::reverse_iterator;

  // Standard iterator interface
  iterator begin() { return map_view_.begin(); }
  iterator end() { return map_view_.end(); }
  const_iterator begin() const { return const_map_view_.begin(); }
  const_iterator end() const { return const_map_view_.end(); }

  // Reverse iterators
  reverse_iterator rbegin() { return map_view_.rbegin(); }
  reverse_iterator rend() { return map_view_.rend(); }
  const_reverse_iterator rbegin() const { return const_map_view_.rbegin(); }
  const_reverse_iterator rend() const { return const_map_view_.rend(); }

 private:
  std::unique_ptr<T[]> map_elements_;
  const std::span<T> map_view_;
  const std::span<const T> const_map_view_;
};

}  // namespace motorium::model
