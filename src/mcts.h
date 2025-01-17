#ifndef MCTS_SRC_MCTS_H_
#define MCTS_SRC_MCTS_H_

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

constexpr uint16_t kWidth = 128;
constexpr uint16_t kHeight = 128;

struct State {
  State(int x, int y,
        std::shared_ptr<std::array<std::array<bool, kWidth>, kHeight>> g)
      : loc_x(x), loc_y(y), grid(std::move(g)) {}

  bool operator<(const State &other) const {
    return std::tie(loc_x, loc_y, grid) <
           std::tie(other.loc_x, other.loc_y, other.grid);
  }

  int loc_x, loc_y;
  std::shared_ptr<std::array<std::array<bool, kWidth>, kHeight>> grid;
};

struct MCTSNode {
  explicit MCTSNode(const State &state, MCTSNode *parent = nullptr)
      : state_(state), parent_(parent), visits_(0), total_reward_(0.0) {}

  double UCT(double exploration_const = 1.41) const {
    if (visits_ == 0) {
      return std::numeric_limits<double>::max();
    }
    return (total_reward_ / visits_) +
           exploration_const * std::sqrt(std::log(parent_->visits_) / visits_);
  }

  State state_;
  MCTSNode *parent_;
  std::vector<std::unique_ptr<MCTSNode>> children_;
  int visits_;
  double total_reward_;
};

class MCTS {
public:
  explicit MCTS(const State &init_state)
      : root_(std::make_unique<MCTSNode>(init_state)),
        rng_(std::random_device{}()) {}

  void Search(int iters);
  MCTSNode *BestChild() const;

private:
  std::unique_ptr<MCTSNode> root_;
  std::mt19937 rng_;
  const int kGoalX = kWidth - 1;
  const int kGoalY = 0;

  MCTSNode *Select(MCTSNode *node);
  void Expand(MCTSNode *node);
  std::pair<std::vector<State>, double> Simulate(State state);
  void Backpropagate(MCTSNode *node, double reward);
  std::vector<State> GetNextStates(const State &state);
  State RandomNextState(const State &state);
  void UpdateObjects(State &state);

  bool IsGoal(const State &state) const {
    return state.loc_x == kGoalX && state.loc_y == kGoalY;
  }
  bool IsCollision(const State &state) const {
    return state.grid->at(state.loc_y).at(state.loc_x);
  }
};

#endif // MCTS_SRC_MCTS_H_
