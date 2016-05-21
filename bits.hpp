#include <array>
#include <numeric>

template <size_t N>
class PathBitsImpl {
 public:
  PathBitsImpl(const uint64_t num) {
    for (size_t i = 0; i < 8; ++i) {
      data[i] = num >> (i*8);
    }
    for (size_t i = 8; i < (N+7)/8; ++i) {
      data[i] = 0;
    }
  }
  PathBitsImpl() : data{} {}
  void set(const size_t index) {
    data[index/8] |= 1 << (index % 8);
  }
  bool test(const size_t index) const {
    return (data[index>>3] >> (index & 7)) & 1;
  }
  size_t count() const {
    size_t cnt = 0;
    for (size_t i = 0; i < (N+7)/8; ++i) {
      for (size_t j = 0; j < 8; ++j) {
        if ((data[i] >> j) & 1) ++cnt;
      }
    }
    return cnt;
  }
  friend size_t std::hash<PathBitsImpl<N>>::operator()(const PathBitsImpl<N> &) const;
  friend bool operator==(const PathBitsImpl<N> &, const PathBitsImpl<N> &);
 private:
  std::array<uint8_t, (N+7)/8> data;
};

namespace std {

template<size_t N>
class hash<PathBitsImpl<N>> {
 public:
  size_t operator()(const PathBitsImpl<N> &bits) const {
    size_t res = 0;
    size_t x = 1;
    for (size_t i = 0; i < (N+7)/8; ++i) {
      res += bits.data[i] * x;
      x *= 17;
    }
    return res;
  }
};

} // namespace std

