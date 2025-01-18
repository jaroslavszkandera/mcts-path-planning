#include "grid.h"
#include "mcts.h"

#include <cstdlib>
#include <ctime>
#include <iostream>

#include <QApplication>
#include <QTimer>

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

  int search_time_ms = 32;

  if (argc > 1) {
    char *end;
    int64_t arg_value = std::strtol(argv[1], &end, 10);

    if (*end == '\0' && arg_value > 0) {
      search_time_ms = static_cast<int>(arg_value);
    } else {
      std::cerr << "Invalid search_time_ms argument. Using default: "
                << search_time_ms << " ms.\n";
    }
  } else {
    std::cout << "No search_time_ms argument provided. Using default: "
              << search_time_ms << " ms.\n";
  }

  QGraphicsScene scene;
  CustomGraphicsView view(&scene);
  view.setFixedSize(kWidth * kCellSize + 2, kHeight * kCellSize + 2);
  view.setSceneRect(0, 0, kWidth * kCellSize, kHeight * kCellSize);

  Grid grid(&scene, search_time_ms);
  QObject::connect(&view, &CustomGraphicsView::ResetRequested, &grid,
                   &Grid::ResetGrid);

  QTimer timer;
  constexpr int kRefreshDelayMs = 10;
  QObject::connect(&timer, &QTimer::timeout, &grid, &Grid::UpdateGrid);
  timer.start(kRefreshDelayMs);

  view.show();

  return app.exec();
}
