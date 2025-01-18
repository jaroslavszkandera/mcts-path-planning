#ifndef MCTS_SRC_GRID_H_
#define MCTS_SRC_GRID_H_

#include "mcts.h"

#include <QApplication>
#include <QGraphicsRectItem>
#include <QGraphicsView>
#include <QKeyEvent>

class CustomGraphicsView : public QGraphicsView {
  Q_OBJECT

public:
  explicit CustomGraphicsView(QGraphicsScene *scene, QWidget *parent = nullptr)
      : QGraphicsView(scene, parent) {}

protected:
  void keyPressEvent(QKeyEvent *event) override {
    if (event->key() == Qt::Key_Space) {
      emit ResetRequested();
    }
  }
signals:
  void ResetRequested();
};

constexpr int kCellSize = 7;

class Grid : public QObject {
  Q_OBJECT

public:
  explicit Grid(QGraphicsScene *scene, int search_time_ms);

public slots:
  void UpdateGrid();
  void ResetGrid();

private:
  QGraphicsScene *scene_;
  int search_time_ms_;
  int agent_x_, agent_y_;
  bool game_paused_ = false;
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

  QGraphicsTextItem *mesg_item_ = nullptr;
  void ShowMessage(const QString &message, const QColor &color);
  void InitializeObjects();
  void GenerateRandomShape(int start_x, int start_y, int size,
                           std::vector<std::pair<int, int>> &cells);
  void RenderGrid();
};

#endif // MCTS_SRC_GRID_H_
