#pragma once

#include <array>
#include <cassert>
#include <iostream>
#include <vector>

namespace utils {

template <typename T>
T sum(const std::vector<T>& v) {
  T sum = 0;
  for (const auto& e : v) sum += e;
  return sum;
}

template <typename T>
T diffSum(const std::vector<T>& v1, const std::vector<T>& v2) {
  assert(v1.size() == v2.size());
  T sum = 0;
  for (size_t i = 0; i < v1.size(); ++i) {
    sum += (v1[i] - v2[i]);
  }
  return sum;
}

template <typename T, size_t Size>
class StableStorage {
 public:
  StableStorage() : data_(), next_free_(0) {}

  size_t add() {
    data_[next_free_] = T();
    next_free_++;
    assert(next_free_ < Size);
    return next_free_ - 1;
  }

  T& at(const size_t& idx) {
    assert(idx <= next_free_);
    return data_[idx];
  }

  auto begin() const { return data_.begin(); }

  auto end() const { return data_.end(); }

 private:
  std::array<T, Size> data_;
  size_t next_free_;
};

template <typename T>
class CartesianProduct {
 public:
  CartesianProduct() : v_(), idxs_({0}), ends_({0}) {}
  explicit CartesianProduct(const std::vector<std::vector<T>>& v)
      : v_(v), idxs_(v.size(), 0), ends_(v.size(), 0) {
    for (size_t i = 0; i < v_.size(); ++i) {
      assert(!v_[i].empty());
      ends_[i] = v_[i].size() - 1;
    }
    assert(!ends_.empty());
    ends_.back()++;

    //     std::cout << "idxs: ";
    //     for (const auto& e : idxs_) {
    //       std::cout << e << ' ';
    //     }
    //     std::cout << std::endl;
  }

  std::vector<T> getCurrent() const {
    std::vector<T> result;
    result.reserve(v_.size());
    for (size_t i = 0; i < v_.size(); ++i) {
      const size_t index = idxs_.at(i);
      const auto& ref = v_.at(i);
      result.push_back(ref.at(index));
    }
    return result;
  }

  inline std::vector<T> current() const { return getCurrent(); }

  bool atEnd() const { return idxs_.back() == ends_.back(); }

  void increment() {
    idxs_[0]++;
    const int max_itr = static_cast<int>(idxs_.size() - 1);
    for (int i = 0; i < max_itr; ++i) {
      if (idxs_[i] > ends_[i]) {
        idxs_[i] = 0;
        idxs_[i + 1]++;
      } else {
        break;
      }
    }
  }

  std::vector<T> getAndIncrement() {
    auto current = getCurrent();
    increment();
    return current;
  }

 private:
  const std::vector<std::vector<T>> v_;
  std::vector<size_t> idxs_;
  std::vector<size_t> ends_;
};

}  // namespace utils
