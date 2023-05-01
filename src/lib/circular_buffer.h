// Copyright 2022 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iterator>

namespace apollo {
namespace lib {

template<typename T, std::size_t SIZE = 100>
class CircularBuffer {
 public:
  CircularBuffer() : idx_(0), size_(0) {}
  ~CircularBuffer() {}

  void push_back(const T& t);
  void pop_back();

  std::size_t size() const;

  bool empty() const;

  Iterator begin() {}
  Iterator end() {}

  struct Iterator {
   public:
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = T*;
    using reference = T&;
    using iterator_category = std::forward_iterator_tag;

    Iterator() : m_ptr_(nullptr) {}
    Iterator(pointer ptr) : m_ptr_(ptr) {}

    reference operator*() const { return *m_ptr_; }
    pointer operator->() const { return m_ptr_; }

    Iterator& operator++() {
      ++m_ptr_;
      return *this;
    }

    Iterator& operator--() {
      --m_ptr;
      return *this;
    }

    friend bool operator==(const Iterator& lhs, const Iterator& rhs) {
      return lhs.m_ptr_ == rhs.m_ptr_;
    }

    friend bool operator!=(const Iterator& lhs, const Iterator& rhs) {
      return lhs.m_ptr_ != rhs.m_ptr_;
    }
  };

 private:
  std::size_t idx_;
  std::size_t size_;
  T arr[SIZE];
};



}  // namespace lib
}  // namespace apollo
