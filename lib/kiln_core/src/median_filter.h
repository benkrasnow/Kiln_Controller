// Small running median filter for rejecting single-sample noise spikes.
#pragma once

#include <cmath>
#include <cstddef>

namespace kiln {

template <size_t N>
class MedianFilter {
  static_assert(N % 2 == 1, "use an odd window");

 public:
  void reset() { count_ = 0; next_ = 0; }

  float push(float v) {
    buf_[next_] = v;
    next_ = (next_ + 1) % N;
    if (count_ < N) ++count_;
    return value();
  }

  // Median of the samples seen so far (NAN if none).
  float value() const {
    if (count_ == 0) return NAN;
    float s[N];
    for (size_t i = 0; i < count_; ++i) s[i] = buf_[i];
    for (size_t i = 1; i < count_; ++i) {  // insertion sort, N is tiny
      const float x = s[i];
      size_t j = i;
      for (; j > 0 && s[j - 1] > x; --j) s[j] = s[j - 1];
      s[j] = x;
    }
    return s[count_ / 2];
  }

 private:
  float buf_[N] = {};
  size_t count_ = 0;
  size_t next_ = 0;
};

}  // namespace kiln
