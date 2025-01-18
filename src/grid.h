#ifndef MCTS_SRC_GRID_H_
#define MCTS_SRC_GRID_H_

#include "mcts.h"

#include <QApplication>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsView>

constexpr int kCellSize = 5;

class Grid : public QObject {
  Q_OBJECT

public:
  explicit Grid(QGraphicsScene *scene);

public slots:
  void UpdateGrid();

private:
  QGraphicsScene *scene_;
  int agent_x_, agent_y_;
  std::array<std::array<bool, kWidth>, kHeight> grid_;
  MCTSNode *best_move_node_;
  QGraphicsRectItem *goal_item_ = nullptr;
  QGraphicsRectItem *agent_item_ = nullptr;

  std::chrono::steady_clock::time_point last_update_time_;
  int frame_count_ = 0;
  int64_t elapsed_time_ = 0.0;

  struct Object {
    std::vector<std::pair<int, int>> cells;
    int vel_x, vel_y;
  };
  std::vector<Object> objects_;
  std::vector<QGraphicsRectItem *> rect_items_;

  void InitializeObjects();
  void GenerateRandomShape(int start_x, int start_y, int size,
                           std::vector<std::pair<int, int>> &cells);
  void RenderGrid();
};

#endif // MCTS_SRC_GRID_H_
