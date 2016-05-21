#include <algorithm>
#include <atomic>
#include <bitset>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>
#include "unistd.h"
//#include <gperftools/profiler.h>
#include "bits.hpp"

constexpr int INF = 20000;

constexpr int GRID_SIZE = 7;
constexpr int PATH_SIZE = 2 * GRID_SIZE * (GRID_SIZE - 1);

constexpr int de[] = {0, GRID_SIZE-1, -1, -GRID_SIZE};
constexpr int dx[] = {1, 0, -1, 0};
constexpr int dy[] = {0, 1, 0, -1};

using PathBits = PathBitsImpl<PATH_SIZE>;

bool is_simple(const PathBits &bits) {
  for (int x = 0; x < GRID_SIZE; ++x) {
    for (int y = 0; y < GRID_SIZE; ++y) {
      int cnt = 0;
      for (int i = 0; i < 4; ++i) {
        if (i - cnt > 1) break;
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) continue;
        int ne = x + 13 * y + de[i];
        if (bits.test(ne)) ++cnt;
      }
      if (cnt >= 3) return false;
    }
  }
  return true;
}

struct Path {
  PathBits bits;
  int8_t x;
  int8_t y;
  bool simple;
  Path() : bits(0), x(0), y(0), simple(true) {}
  Path(const PathBits &bits, const int x, const int y)
    : bits(bits), x(x), y(y), simple(is_simple(bits)) {}
  Path(const Path &) = default;
  Path(Path &&) = default;
  Path &operator=(const Path &) = default;
  Path &operator=(Path &&) = default;
};

bool operator==(const PathBits &lhs, const PathBits &rhs) {
  return lhs.data == rhs.data;
}

bool operator==(const Path &lhs, const Path &rhs) {
  return lhs.bits == rhs.bits && lhs.x == rhs.x && lhs.y == rhs.y;
}
    
struct AffineFunc {
  int a1, a0;
  AffineFunc(const int linear_factor, const int constant_factor)
    : a1(linear_factor), a0(constant_factor) {}
  int operator()(const int val) const {
    return a1 * val + a0;
  }
  AffineFunc operator()(const AffineFunc &func) const {
    return AffineFunc(a1 * func.a1, a1 * func.a0 + a0);
  }
};

struct PathWithScore {
  Path path;
  int16_t score;
  int16_t lower_bound;
  int16_t upper_bound;
  PathWithScore(const Path &path, const int score)
    : path(path), score(score), lower_bound(-INF), upper_bound(INF) {}
  PathWithScore(const PathWithScore &) = default;
  PathWithScore(PathWithScore &&) = default;
  PathWithScore &operator=(const PathWithScore &) = default;
  PathWithScore &operator=(PathWithScore &&) = default;
};

bool operator<(const PathWithScore &lhs, const PathWithScore &rhs) {
  return lhs.lower_bound < rhs.lower_bound;
}

template <typename T>
using Table = std::vector<std::vector<T>>;

template <typename T>
Table<T> make_table(const int h, const int w, const T &val) {
  return Table<T>(h, std::vector<T>(w, val));
}

template <typename T>
Table<T> make_table(const int h, const int w) {
  Table<T> res;
  for (int i = 0; i < h; ++i) res.emplace_back(w);
  return res;
}

struct Range {
  int lower_bound;
  int upper_bound;
  bool is_reachable;
};

Range get_range(const PathWithScore &ps, const std::vector<AffineFunc> &edges) {
  using table_b = std::vector<std::bitset<GRID_SIZE>>;
  const Path &path = ps.path;
  table_b table(GRID_SIZE, std::bitset<GRID_SIZE>(0));
  table_b visit(GRID_SIZE, std::bitset<GRID_SIZE>(0));
  table[path.y].set(path.x);
  Table<int> lb = make_table(GRID_SIZE, GRID_SIZE, -INF);
  lb[path.x][path.y] = ps.score;
  using P = std::pair<int, int>;
  std::queue<P> q;
  q.emplace(path.x, path.y);
  PathBits enable_edges(0);
  while(!q.empty()) {
    int x, y;
    std::tie(x, y) = q.front(); q.pop();
    visit[x].set(y);
    for (int i = 0; i < 4; ++i) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      int ne = x + 13 * y + de[i];
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) continue;
      if (path.bits.test(ne)) continue;
      enable_edges.set(ne);
      if (visit[nx].test(ny)) continue;
      lb[nx][ny] = std::max(lb[nx][ny], edges[ne](lb[x][y]));
      if (table[nx].test(ny)) continue;
      table[nx].set(ny);
      q.emplace(nx, ny);
    }
  }
  int mul = 1, add = ps.score;
  for (int i = 0; i < PATH_SIZE; ++i) {
    if (enable_edges.test(i)) {
      mul *= edges[i].a1;
      add += std::max(0, edges[i].a0);
    }
  }
  return {
    lb.back().back(),
    mul * add,
    table.back().test(GRID_SIZE-1)
  };
}

template <typename T, typename U>
class TSUMM { // Thread-Safe Unordered-Max-Map
 public:
  bool update(const T &key, const U &val) {
    std::lock_guard<std::mutex> lg(m);
    if (!um.count(key) || val > um[key]) {
      um[key] = val;
      return true;
    } else {
      return false;
    }
  }
  void force_update(const T &key, const U &val) {
    std::lock_guard<std::mutex> lg(m);
    um[key] = val;
  }
  U get(const T &key) {
    std::lock_guard<std::mutex> lg(m);
    return um[key];
  }
 private:
  std::unordered_map<T, U> um;
  std::mutex m;
};

template <typename T>
class TSPQ { // Thread-Safe Priority Queue
 public:
  void push(const T &val) {
    std::lock_guard<std::mutex> lg(m);
    que.push(val);
  }
  boost::optional<T> try_pop() {
    std::lock_guard<std::mutex> lg(m);
    if (que.empty()) return boost::none;
    T val = que.top();
    que.pop();
    return val;
  }
 private:
  std::priority_queue<T> que;
  std::mutex m;
};

class MaxScore {
 public:
  MaxScore(int init_score) : score(init_score) {}
  bool update(const int new_score) {
    int old_score = score;
    bool updated = new_score > old_score;
    while (old_score < new_score &&
        !score.compare_exchange_weak(old_score, new_score));
    return updated;
  }
  explicit operator int() const {
    return score;
  }
 private:
  std::atomic<int> score;
};

void worker(const std::vector<AffineFunc> &edges, TSPQ<PathWithScore> &que, Table<TSUMM<PathBits, int>> &memo, MaxScore &max_score) {
  while (const auto ps_opt = que.try_pop()) {
    const auto ps = *ps_opt;
    int x = ps.path.x;
    int y = ps.path.y;
    if (!ps.path.simple && ps.score < memo[x][y].get(ps.path.bits)) continue;
    if (ps.upper_bound < static_cast<int>(max_score)) continue;
    if (max_score.update(ps.lower_bound)) {
      std::cerr << ps.lower_bound << ' ' << ps.score << ' ' << ps.upper_bound
        << ", length: " << ps.path.bits.count()
        << std::endl;
    }
    for (int i = 0; i < 4; ++i) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) continue;
      int ne = x + (2 * GRID_SIZE - 1) * y + de[i];
      if (ps.path.bits.test(ne)) continue;
      PathBits pb = ps.path.bits;
      pb.set(ne);
      Path np(pb, nx, ny);
      int ns = edges[ne](ps.score);
      if (np.simple || memo[nx][ny].update(pb, ns)) {
        PathWithScore ps(np, ns);
        Range range = get_range(ps, edges);
        if (range.is_reachable) {
          ps.lower_bound = range.lower_bound;
          ps.upper_bound = range.upper_bound;
          que.push(ps);
        } else if (!np.simple) {
          memo[nx][ny].force_update(pb, INF);
        }
      }
    }
  }
}

int main() {
  std::cerr << sizeof(Path) << std::endl;
  std::cerr << sizeof(PathWithScore) << std::endl;
  std::vector<AffineFunc> edges;
  for (int i = 0; i < PATH_SIZE; ++i) {
    std::string edge;
    std::cin >> edge;
    switch (edge[0]) {
      case '+':
        edges.emplace_back(1, edge[1] - '0');
        break;
      case '-':
        edges.emplace_back(1, -(edge[1] - '0'));
        break;
      case '*':
        edges.emplace_back(edge[1] - '0', 0);
        break;
      default:
        std::cerr << i << " error!!" << std::endl;
        return -1;
    }
  }
  Table<TSUMM<PathBits, int>> memo = make_table<TSUMM<PathBits, int>>(GRID_SIZE, GRID_SIZE);
  TSPQ<PathWithScore> que;
  memo[0][0].update(PathBits(), 1);
  que.push(PathWithScore(Path(), 1));
  MaxScore max_score(-INF);
  std::vector<std::thread> vt;
  //ProfilerStart("paper.prof");
  for (size_t i = 0; i < std::thread::hardware_concurrency(); ++i) {
    vt.emplace_back(worker, std::cref(edges), std::ref(que), std::ref(memo), std::ref(max_score));
    sleep(1);
  }
  for (auto &&t : vt) {
    t.join();
  }
  //ProfilerStop();
  std::cout << "result: " << static_cast<int>(max_score) << std::endl;
  return 0;
}
