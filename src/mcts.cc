#include "mcts.h"

#include <algorithm>
#include <iostream>

void MCTS::Search(int iters) {
  for (int i = 0; i < iters; ++i) {
    MCTSNode *selected = Select(root_.get());
    double reward = Simulate(selected->state_);
    Backpropagate(selected, reward);
  }
}

MCTSNode *MCTS::BestChild() const {
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

MCTSNode *MCTS::Select(MCTSNode *node) {
  while (!node->children_.empty()) {
    node = std::max_element(
               node->children_.begin(), node->children_.end(),
               [](const auto &a, const auto &b) { return a->UCT() < b->UCT(); })
               ->get();
  }
  if (node->visits_ == 0) {
    return node;
  }
  Expand(node);
  return node->children_.empty() ? node : node->children_.front().get();
}

void MCTS::Expand(MCTSNode *node) {
  const std::vector<State> next_states = GetNextStates(node->state_);
  for (const auto &state : next_states) {
    node->children_.emplace_back(std::make_unique<MCTSNode>(state, node));
  }
}

double MCTS::Simulate(State state) {
  double progress_reward = -std::sqrt(std::pow(state.loc_x - kGoalX, 2) +
                                      std::pow(state.loc_y - kGoalY, 2));

  double reward = progress_reward;
  double heavy_penalty = -100;
  int num_steps = 20;
  for (int step = 0; step < num_steps; ++step) {
    if (IsCollision(state)) {
      return -1000.0;
    }

    double avoidance_reward = 0.0;
    for (uint16_t y = std::max(0, state.loc_y - num_steps);
         y < std::min(static_cast<uint16_t>(state.loc_y + num_steps), kHeight);
         ++y) {
      for (uint16_t x = std::max(0, state.loc_x - num_steps);
           x < std::min(static_cast<uint16_t>(state.loc_x + num_steps), kWidth);
           ++x) {
        if (state.grid->at(y).at(x)) {
          double dist_to_obstacle = std::sqrt(std::pow(state.loc_x - x, 2) +
                                              std::pow(state.loc_y - y, 2));
          if (dist_to_obstacle <= 0.7) {
            avoidance_reward += heavy_penalty;
          }
          // Penalty for object proximity
          if (dist_to_obstacle < 10.0) {
            avoidance_reward -= 1.0 / dist_to_obstacle;
          }
        }
      }
    }

    reward += progress_reward + avoidance_reward;
    if (IsGoal(state)) {
      return 1000.0;
    }

    state = RandomNextState(state);
    progress_reward = -std::sqrt(std::pow(state.loc_x - kGoalX, 2) +
                                 std::pow(state.loc_y - kGoalY, 2));
  }
  // std::cout << "reward " << reward << "\n";
  return reward;
}

void MCTS::Backpropagate(MCTSNode *node, double reward) {
  while (node) {
    node->visits_ += 1;
    node->total_reward_ += reward;
    node = node->parent_;
  }
}

std::vector<State> MCTS::GetNextStates(const State &state) {
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

State MCTS::RandomNextState(const State &state) {
  auto next_states = GetNextStates(state);
  if (next_states.empty()) {
    return state;
  }
  std::uniform_int_distribution<int> dist(
      0, static_cast<int>(next_states.size()) - 1);
  return next_states[dist(rng_)];
}

void MCTS::UpdateObjects(State &state) {
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
