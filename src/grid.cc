#include "grid.h"

#include <iostream>
#include <queue>

Grid::Grid(QGraphicsScene *scene)
    : scene_(scene), agent_x_(0), agent_y_(kHeight - 1),
      last_update_time_(std::chrono::steady_clock::now()) {
  std::srand(static_cast<unsigned>(std::time(nullptr)));
  for (auto &row : grid_) {
    row.fill(false);
  }
  InitializeObjects();
  RenderGrid();
}

void Grid::UpdateGrid() {
  const auto kNow = std::chrono::steady_clock::now();
  const auto kDeltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                              kNow - last_update_time_)
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

void Grid::InitializeObjects() {
  constexpr int kNumObjects = 200;
  constexpr int kMinSize = 2;
  constexpr int kMaxSize = 20;

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

void Grid::GenerateRandomShape(int start_x, int start_y, int size,
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

void Grid::RenderGrid() {
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
                      static_cast<double>(0) * kCellSize, kCellSize, kCellSize,
                      QPen(Qt::green), QBrush(Qt::green));
  rect_items_.push_back(goal_item);

  QGraphicsRectItem *agent_item =
      scene_->addRect(static_cast<double>(agent_x_) * kCellSize,
                      static_cast<double>(agent_y_) * kCellSize, kCellSize,
                      kCellSize, QPen(Qt::red), QBrush(Qt::red));
  rect_items_.push_back(agent_item);
}
