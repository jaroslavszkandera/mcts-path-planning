#include <QApplication>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QTimer>
#include <array>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <queue>
#include <random>
#include <vector>

constexpr uint16_t kWidth = 128;
constexpr uint16_t kHeight = 128;
constexpr int kCellSize = 5;

struct State {
  int loc_x, loc_y;
  std::shared_ptr<std::array<std::array<bool, kWidth>, kHeight>> grid;

  State(int x, int y,
        std::shared_ptr<std::array<std::array<bool, kWidth>, kHeight>> g)
      : loc_x(x), loc_y(y), grid(std::move(g)) {}

  bool operator<(const State &other) const {
    return std::tie(loc_x, loc_y, grid) <
           std::tie(other.loc_x, other.loc_y, other.grid);
  }
};

class MCTSNode {
public:
  State state_;
  MCTSNode *parent_;
  std::vector<std::unique_ptr<MCTSNode>> children_;
  int visits_;
  double total_reward_;

  explicit MCTSNode(const State &state, MCTSNode *parent = nullptr)
      : state_(state), parent_(parent), visits_(0), total_reward_(0.0) {}

  double UCT(double exploration_const = 1.41) const {
    if (visits_ == 0) {
      return std::numeric_limits<double>::max();
    }
    return (total_reward_ / visits_) +
           exploration_const * std::sqrt(std::log(parent_->visits_) / visits_);
  }
};

class MCTS {
public:
  std::unique_ptr<MCTSNode> root_;
  std::mt19937 rng_;

  explicit MCTS(const State &init_state)
      : root_(std::make_unique<MCTSNode>(init_state)),
        rng_(std::random_device{}()) {}

  void Search(int iters) {
    for (int i = 0; i < iters; ++i) {
      MCTSNode *selected = Select(root_.get());
      // TODO(JS): Display path
      auto [path, reward] = Simulate(selected->state_);
      Backpropagate(selected, reward);
    }
  }
  MCTSNode *BestChild() const {
    if (!root_) {
      std::cerr << "Error: root_ is null in BestChild()\n";
      return nullptr;
    }
    if (root_->children_.empty()) {
      std::cerr << "Error: No children in root_->children_\n";
      return nullptr;
    }
    return std::max_element(root_->children_.begin(), root_->children_.end(),
                            [](const auto &a, const auto &b) {
                              return a->visits_ < b->visits_;
                            })
        ->get();
  }

private:
  const int kGoalX = kWidth - 1;
  const int kGoalY = 0;

  MCTSNode *Select(MCTSNode *node) {
    while (!node->children_.empty()) {
      node = std::max_element(node->children_.begin(), node->children_.end(),
                              [](const auto &a, const auto &b) {
                                return a->UCT() < b->UCT();
                              })
                 ->get();
    }
    if (node->visits_ == 0) {
      return node;
    }
    Expand(node);
    return node->children_.empty() ? node : node->children_.front().get();
  };

  void Expand(MCTSNode *node) {
    const std::vector<State> next_states = GetNextStates(node->state_);
    for (const auto &state : next_states) {
      node->children_.emplace_back(std::make_unique<MCTSNode>(state, node));
    }
  };

  std::pair<std::vector<State>, double> Simulate(State state) {
    std::vector<State> path;
    path.push_back(state);
    double progress_reward = -std::sqrt(std::pow(state.loc_x - kGoalX, 2) +
                                        std::pow(state.loc_y - kGoalY, 2));

    for (int step = 0; step < 10; ++step) {
      if (IsCollision(state)) {
        return {path, -100.0};
      }
      if (IsGoal(state)) {
        return {path, 100.0};
      }

      state = RandomNextState(state);
      path.push_back(state);
      progress_reward = -std::sqrt(std::pow(state.loc_x - kGoalX, 2) +
                                   std::pow(state.loc_y - kGoalY, 2));
    }
    return {path, progress_reward};
  }

  void Backpropagate(MCTSNode *node, double reward) {
    while (node) {
      node->visits_ += 1;
      node->total_reward_ += reward;
      node = node->parent_;
    }
  }

  std::vector<State> GetNextStates(const State &state) {
    const std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
    std::vector<State> next_states;
    for (const auto &[dx, dy] : directions) {
      int nx = state.loc_x + dx;
      int ny = state.loc_y + dy;
      if (nx >= 0 && nx < static_cast<int>(kWidth) && ny >= 0 &&
          ny < static_cast<int>(kHeight) && !state.grid->at(ny).at(nx)) {
        State new_state(nx, ny, state.grid);
        next_states.push_back(new_state);
      }
    }
    return next_states;
  }

  State RandomNextState(const State &state) {
    auto next_states = GetNextStates(state);
    if (next_states.empty()) {
      return state;
    }
    std::uniform_int_distribution<int> dist(0, next_states.size() - 1);
    return next_states[dist(rng_)];
  }

  bool IsGoal(const State &state) const {
    return state.loc_x == kGoalX && state.loc_y == kGoalY;
  }

  bool IsCollision(const State &state) const {
    return state.grid->at(state.loc_y).at(state.loc_x);
  }

  void UpdateObjects(State &state) {
    for (uint16_t y = 0; y < kHeight; ++y) {
      for (uint16_t x = 0; x < kWidth; ++x) {
        if (state.grid->at(y).at(x) && std::rand() % 10 < 3) {
          state.grid->at(y).at(x) = false;
          int nx = (x + std::rand() % 3 - 1 + kWidth) % kWidth;
          int ny = (y + std::rand() % 3 - 1 + kHeight) % kHeight;
          state.grid->at(ny).at(nx) = true;
        }
      }
    }
  }
};

class Grid : public QObject {
  Q_OBJECT

public:
  explicit Grid(QGraphicsScene *scene)
      : scene_(scene), agent_x_(0), agent_y_(kHeight - 1),
        last_update_time_(std::chrono::steady_clock::now()) {
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    for (auto &row : grid_) {
      row.fill(false);
    }
    InitializeObjects();
    RenderGrid();
  }

public slots:
  void UpdateGrid() {
    const auto kNow = std::chrono::steady_clock::now();
    const auto kDeltaTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(kNow -
                                                              last_update_time_)
            .count();
    last_update_time_ = kNow;

    elapsed_time_ += kDeltaTime;
    frame_count_++;

    if (elapsed_time_ >= 1000) {
      double fps = frame_count_ * 1000.0 / static_cast<double>(elapsed_time_);
      std::cout << "FPS: " << fps << std::endl;
      frame_count_ = 0;
      elapsed_time_ = 0.0;
    }

    for (auto &row : grid_) {
      row.fill(false);
    }

    for (auto &object : objects_) {
      for (auto &cell : object.cells) {
        cell.first += object.vel_x;
        cell.second += object.vel_y;

        if (cell.first >= static_cast<int>(kWidth)) {
          cell.first = 0;
        }
        if (cell.first < 0) {
          cell.first = static_cast<int>(kWidth) - 1;
        }
        if (cell.second >= static_cast<int>(kHeight)) {
          cell.second = 0;
        }
        if (cell.second < 0) {
          cell.second = static_cast<int>(kHeight) - 1;
        }

        grid_[cell.second][cell.first] = true;
      }
    }

    State curr_state(
        agent_x_, agent_y_,
        std::make_shared<std::array<std::array<bool, kWidth>, kHeight>>(grid_));
    MCTS mcts(curr_state);
    mcts.Search(500);

    best_move_node_ = mcts.BestChild();
    if (best_move_node_) {
      agent_x_ = best_move_node_->state_.loc_x;
      agent_y_ = best_move_node_->state_.loc_y;
    } else {
      std::cerr << "No valid moves found!\n";
      QApplication::quit();
      return;
    }

    if (best_move_node_->state_.loc_x == kWidth - 1 &&
        best_move_node_->state_.loc_y == 0) {
      std::cout << "Success!\n";
      QApplication::quit();
      return;
    }

    RenderGrid();
  }

private:
  std::chrono::steady_clock::time_point last_update_time_;
  int frame_count_ = 0;
  int64_t elapsed_time_ = 0.0;

  QGraphicsScene *scene_;
  std::array<std::array<bool, kWidth>, kHeight> grid_;
  int agent_x_, agent_y_;
  MCTSNode *best_move_node_;

  struct Object {
    std::vector<std::pair<int, int>> cells;
    int vel_x, vel_y;
  };

  std::vector<Object> objects_;
  std::vector<QGraphicsRectItem *> rect_items_;

  void InitializeObjects() {
    constexpr int kNumObjects = 200;
    constexpr int kMinSize = 4;
    constexpr int kMaxSize = 10;

    for (int i = 0; i < kNumObjects; ++i) {
      Object object;
      int start_x = std::rand() % static_cast<int>(kWidth);
      int start_y = std::rand() % static_cast<int>(kHeight);
      int size = kMinSize + std::rand() % (kMaxSize - kMinSize + 1);
      GenerateRandomShape(start_x, start_y, size, object.cells);

      // Randomize direction -1, 0, or 1
      do {
        object.vel_x = std::rand() % 3 - 1;
        object.vel_y = std::rand() % 3 - 1;
      } while (object.vel_x == 0 && object.vel_y == 0);

      objects_.push_back(object);

      for (const auto &cell : object.cells) {
        grid_[cell.second][cell.first] = true;
      }
    }
  }

  void GenerateRandomShape(int start_x, int start_y, int size,
                           std::vector<std::pair<int, int>> &cells) {
    std::queue<std::pair<int, int>> queue;
    queue.push({start_x, start_y});
    cells.push_back({start_x, start_y});

    while (static_cast<int>(cells.size()) < size) {
      auto [x, y] = queue.front();
      queue.pop();

      const std::vector<std::pair<int, int>> neighbors = {
          {x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
      for (const auto &neighbor : neighbors) {
        int nx = neighbor.first;
        int ny = neighbor.second;

        if (nx >= 0 && nx < static_cast<int>(kWidth) && ny >= 0 &&
            ny < static_cast<int>(kHeight) &&
            std::find(cells.begin(), cells.end(), neighbor) == cells.end()) {
          cells.push_back(neighbor);
          queue.push(neighbor);

          if (static_cast<int>(cells.size()) == size) {
            break;
          }
        }
      }
    }
  }

  void RenderGrid() {
    for (auto *item : rect_items_) {
      scene_->removeItem(item);
      delete item;
    }
    rect_items_.clear();

    for (size_t y = 0; y < kHeight; ++y) {
      for (size_t x = 0; x < kWidth; ++x) {
        QGraphicsRectItem *cell = scene_->addRect(
            static_cast<double>(x) * kCellSize,
            static_cast<double>(y) * kCellSize, kCellSize, kCellSize,
            QPen(Qt::black), QBrush(grid_[y][x] ? Qt::black : Qt::white));
        rect_items_.push_back(cell);
      }
    }

    QGraphicsRectItem *goal_item =
        scene_->addRect(static_cast<double>(kWidth - 1) * kCellSize,
                        static_cast<double>(0) * kCellSize, kCellSize,
                        kCellSize, QPen(Qt::green), QBrush(Qt::green));
    rect_items_.push_back(goal_item);

    QGraphicsRectItem *agent_item =
        scene_->addRect(static_cast<double>(agent_x_) * kCellSize,
                        static_cast<double>(agent_y_) * kCellSize, kCellSize,
                        kCellSize, QPen(Qt::red), QBrush(Qt::red));
    rect_items_.push_back(agent_item);
  }
};

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

  QGraphicsScene scene;
  QGraphicsView view(&scene);
  view.setFixedSize(kWidth * kCellSize + 2, kHeight * kCellSize + 2);
  view.setSceneRect(0, 0, kWidth * kCellSize, kHeight * kCellSize);

  Grid grid(&scene);

  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, &grid, &Grid::UpdateGrid);
  timer.start(1);

  view.show();

  return app.exec();
}

#include "main.moc"
