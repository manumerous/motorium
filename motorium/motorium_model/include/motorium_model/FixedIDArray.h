#pragma once

#include <Eigen/Dense>
#include <concepts>
#include <memory>
#include <span>

#include <motorium_core/Check.h>

/**
 * Fixed-size, contiguous container indexed by dense zero-based IDs optimized for
 * realtime use: provides bounds-checked access, no dynamic reallocation after
 * construction, standard iteration, and efficient extraction to Eigen vectors.
 *
 * @tparam T Element type stored for each ID.
 */

/**
 * Construct a FixedIDArray with the given number of elements.
 *
 * The container allocates storage for exactly `size` elements and exposes them
 * via contiguous iteration views.
 *
 * @param size Number of elements to allocate; must be greater than 0 and at most 255.
 */

/**
 * Access the element for the given ID with bounds checking.
 *
 * @param element_id Zero-based ID of the element to access.
 * @returns Reference to the element associated with `element_id`.
 */

/**
 * Convert a sequence of IDs into an Eigen dynamic column vector using an extractor.
 *
 * The extractor is invoked for each stored element referenced by the IDs in
 * `range` to produce a scalar of type `ScalarType`. If `T` provides a
 * `has_value()` method (optional-like) and an element is empty, `default_value`
 * is used for that position.
 *
 * @tparam Range Any range of integer IDs (e.g., container or view).
 * @tparam ScalarType Scalar type of the resulting Eigen vector.
 * @param range Range of IDs to read, iteration order determines vector order.
 * @param extractor Callable that maps a `const T&` (or `const T::value_type&` when optional-like) to `ScalarType`.
 * @param default_value Value to use when an optional-like element is not present; defaults to `ScalarType{}`.
 * @returns Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> Column vector with one entry per ID in `range`.
 */
namespace motorium::model {

// The FixedIDArray is a lightweight data container indexed by dense, zero-based
// IDs. By preventing operations leading to dynamic memory allocating and
// providing safe, bounds-checked access, standard iteration, and cache-friendly
// contiguous storage and extraction of per-ID data into Eigen vectors it is
// optimized for realtime programming.

// Define an extractor that gets a scalar value out of T for eigen vectorization
// when T is itself a composite type (e.g. struct).
template <typename E, typename T, typename ScalarType>
concept IDMapExtractor = requires(E e, const T &val) {
  { e(val) } -> std::same_as<ScalarType>;
};

template <typename T> class FixedIDArray {
public:
  explicit FixedIDArray(size_t size)
      : map_elements_(new T[size]), map_view_(map_elements_.get(), size),
        const_map_view_(map_elements_.get(), size) {
    // Limit map size
    MT_CHECK(size <= 255)
        << "Too many elements in ID map. max supported size is 255";

    MT_CHECK(size != 0) << "Cannot initialize empty map!";
  }
  FixedIDArray() = delete;

  FixedIDArray(const FixedIDArray<T> &other) = delete;
  FixedIDArray(FixedIDArray<T> &&other) = delete;
  FixedIDArray &operator=(FixedIDArray<T> &&other) =
      delete; // Disabled for now. Needs extra care with spans.
  FixedIDArray &operator=(const FixedIDArray<T> &other) {
    // "same type" is enforced by the signature (FixedIDArray<T>).
    // Still check length to ensure maps are compatible.
    MT_CHECK(this->size() == other.size()) << std::format(
        "FixedIDArray::assign size mismatch: this.size()={}, other.size()={}",
        this->size(), other.size());

    const size_t n = this->size();
    for (size_t i = 0; i < n; ++i) {
      map_elements_[i] = other.map_elements_[i];
    }
    return *this;
  }

  bool contains(size_t id) const noexcept { return id < map_view_.size(); }

  size_t size() const { return map_view_.size(); }

  T &at(size_t element_id) {
    MT_CHECK(this->contains(element_id)) << std::format(
        "Element with ID {} not found in IDMap (Max ID for this map is: "
        "{})",
        element_id, map_view_.size() - 1);

    return map_elements_[element_id];
  }

  const T &at(size_t element_id) const {
    MT_CHECK(this->contains(element_id)) << std::format(
        "Element with ID {} not found in IDMap (Max ID for this map is: "
        "{})",
        element_id, map_view_.size() - 1);

    return map_elements_[element_id];
  }

  T &operator[](size_t element_id) { return at(element_id); }

  const T &operator[](size_t element_id) const { return at(element_id); }

  // Uses a default value in case T is a std::optional that is a nullopt.
  template <typename Range, typename ScalarType>
  Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>
  toEigenVector(const Range &range,
                IDMapExtractor<T, ScalarType> auto extractor,
                ScalarType default_value = ScalarType{}) const {
    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> vector(
        std::ranges::size(range));

    size_t index = 0;
    for (const auto &id : range) {
      const T &val = this->at(id);
      if constexpr (requires { val.has_value(); }) {
        // T is optional-like (has has_value method)
        if (val.has_value()) {
          vector(index) = extractor(*val);
        } else {
          vector(index) = default_value;
        }
      } else {
        // T is not optional, always use the value
        vector(index) = extractor(val);
      }
      ++index;
    }
    return vector;
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

} // namespace motorium::model