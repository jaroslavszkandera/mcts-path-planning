#include "grid.h"
#include "mcts.h"

#include <ctime>

#include <QApplication>
#include <QTimer>

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
