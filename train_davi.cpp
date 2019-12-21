#include <iostream>
#include <vector>
#include <array>
#include <random>
#include <limits>
#include <string>
#include <memory>

#include <torch/torch.h>

#include "sbpuzzle.hpp"
#include "search2.hpp"
#include "heuristics.hpp"
#include "sblock_utils.hpp"

const std::string TMP_NET_FILE_PATH = ".train_davi_tmp_net.pt";
const std::string TMP_OPTIM_FILE_PATH = ".train_davi_tmp_optim.pt";
const size_t DEFAULT_NUM_ITERATIONS = 1000;
const size_t DEFAULT_BATCH_SIZE = 1024;
const size_t DEFAULT_SCRAMBLE_MAX = 100;
const unsigned DEFAULT_SEED = 42;
const float DEFAULT_LEARNING_RATE = 0.01;
const float DEFAULT_LOSS_THRESHOLD = 0.05;
const size_t DEFAULT_PRINT_EVERY = 500;

#ifndef __H
#define __H 3
#endif
#ifndef __W
#define __W 3
#endif

using sbpuzzle::psize_t;

constexpr psize_t H = __H, W = __W;
using Puzzle = sbpuzzle::SBPuzzle<H, W>;
using TSA = sbpuzzle::TileSwapAction;
using ManhH = sbpuzzle::ManhattanHeuristic<H, W>;

template <psize_t H, psize_t W, class URNG, class Distribution>
void fill_scrambled_puzzles(size_t num_puzzles, 
                            Distribution &move_distribution,
                            URNG &rng,
                            std::vector<Puzzle> &puzzles_out,
                            std::vector<size_t> &num_moves_out)
{
    // fill a constant tile array with goal state values
    std::array<uint8_t, H*W> goal_tiles;
    std::iota(goal_tiles.begin(), goal_tiles.end(), 0);
    for(size_t i = 0; i < num_puzzles; ++i) {
        // create a goal state puzzle and select a number of moves
        Puzzle puzzle(goal_tiles);
        int64_t num_moves = move_distribution(rng);
        // randomly apply a move to the puzzle num_moves times
        for(int64_t j = 0; j < num_moves; ++j) {
            // TODO: perhaps prevent backtracking of previous move? might help just a little
            std::vector<TSA> actions = puzzle.template possible_actions<TSA>();
            std::uniform_int_distribution<int64_t> action_index_distr(0, actions.size()-1);
            int64_t action_i = action_index_distr(rng);
            puzzle.apply_action(actions[action_i]);
        }
        // add the puzzle
        num_moves_out.emplace_back(num_moves);
        puzzles_out.emplace_back(puzzle);
    }
}

template <typename C>
std::ostream &stream_container(std::ostream &os, const C &c) {
    for(const auto &x : c)
        os << x << " ";
    return os;
}

template <typename Itr>
std::ostream &stream_itr(std::ostream &os, Itr begin, Itr end) {
    while(begin != end)
        os << (*begin++) << " ";
    return os;
}

struct Net : torch::nn::Module {
    Net() : torch::nn::Module(), fc1(nullptr), fc2(nullptr), fc3(nullptr), fc4(nullptr), fc5(nullptr) {
        fc1 = register_module("fc1", torch::nn::Linear(H*W*H*W, 512));
        fc2 = register_module("fc2", torch::nn::Linear(512, 1024));
        fc3 = register_module("fc3", torch::nn::Linear(1024, 256));
        fc4 = register_module("fc4", torch::nn::Linear(256, 64));
        fc5 = register_module("fc5", torch::nn::Linear(64, 1));
    }

    torch::Tensor forward(torch::Tensor x) {
        x = torch::relu(fc1->forward(x));
        x = torch::relu(fc2->forward(x));
        x = torch::relu(fc3->forward(x));
        x = torch::relu(fc4->forward(x));
        x = torch::squeeze(fc5->forward(x));
        return x;
    }

    torch::nn::Linear fc1, fc2, fc3, fc4, fc5;
};

int main() {
    size_t num_iterations = DEFAULT_NUM_ITERATIONS;
    size_t batch_size = DEFAULT_BATCH_SIZE;
    size_t print_every = DEFAULT_PRINT_EVERY;
    size_t scramble_max = DEFAULT_SCRAMBLE_MAX;
    unsigned seed = DEFAULT_SEED;
    float learning_rate = DEFAULT_LEARNING_RATE;
    float loss_threshold = DEFAULT_LOSS_THRESHOLD; // std::numeric_limits<float>::max();

    num_iterations = 100000;

    auto rng = std::default_random_engine(seed);

    Puzzle goal_state = Puzzle::no_mask_goal_state();

    torch::Device device(torch::kCUDA);

    std::shared_ptr<Net> main_net(new Net()), 
                         cur_net(new Net());
    torch::save(main_net, TMP_NET_FILE_PATH);
    torch::load(cur_net, TMP_NET_FILE_PATH);

    main_net->to(device);
    cur_net->to(device);

    torch::optim::Adam cur_optimizer(cur_net->parameters(), 
                                     torch::optim::AdamOptions(learning_rate));

    size_t scramble_top = 2;
    const int64_t S = H * W,
                  Ssq = S * S;
    const size_t num_puzzles = batch_size;
    std::vector<Puzzle> neighbor_puzzles(num_puzzles); 
    std::vector<bool> neighbor_exists(num_puzzles); // marking non-existing neighbors
    std::vector<bool> is_goal_state(num_puzzles);
    std::vector<float> encoded(num_puzzles * Ssq); // for holding one hot encodings of neighbors
    for(size_t step = 0; step < num_iterations; ++step) {
        std::uniform_int_distribution<int64_t> move_distr(1, scramble_top);
        std::vector<Puzzle> puzzles;
        std::vector<size_t> nmoves;
        fill_scrambled_puzzles<H, W>(num_puzzles, move_distr, rng, puzzles, nmoves);

        // After the scrambled states, we have to generate the neighbours
        // of each scrambled state and then pass them through the network
        // for value iteration. How?

        // First, generate the possible actions for every puzzle
        std::vector<std::vector<TSA>> puzzle_actions;
        std::transform(puzzles.begin(), puzzles.end(), std::back_inserter(puzzle_actions),
                [](const Puzzle &p) { return p.template possible_actions<TSA>(); });

        // Then, we have to perform approximate value iteration
        std::vector<float> est_costs(num_puzzles, std::numeric_limits<float>::max());
        // for now, assume the number of max actions is constant
        size_t actions_max = 4;
        // for each possible action
        std::fill(neighbor_exists.begin(), neighbor_exists.end(), true);
        std::fill(is_goal_state.begin(), is_goal_state.end(), false);
        for(size_t ai = 0; ai < actions_max; ++ai) {
            // generate all neighbors with that action,
            // put in dummy value if action does not exist
            // also mark that puzzle for reset on return so that values are not affected
            for(size_t pi = 0; pi < num_puzzles; ++pi) {
                Puzzle p2 = puzzles[pi];
                if(ai < puzzle_actions[pi].size()) { // if an action is available, apply it
                    p2.apply_action(puzzle_actions[pi][ai]);
                    if(p2 == goal_state)
                        is_goal_state[pi] = true;
                }
                else // otherwise, mark for ignoring on return from network
                    neighbor_exists[pi] = false;
                // one hot encode the puzzle
                one_hot_encode<uint8_t, float>(p2.get_tiles().data(), S, 
                        &encoded[pi * Ssq]);
            }

            // TODO: define net, device etc.
            // pass the neighbor puzzles through the main network
            {
                torch::NoGradGuard nograd; // like with torch.no_grad():
                auto inp_tensor = torch::from_blob(encoded.data(), 
                        {static_cast<long>(num_puzzles), Ssq}).to(device);
                auto preds = main_net->forward(inp_tensor).to(torch::kCPU); // to cpu for iterating over
                // put preds back into the est_costs vector since we are value iterating
                auto acc = preds.template accessor<float, 1>();
                for(size_t ti = 0; ti < num_puzzles; ++ti) {
                    if(puzzles[ti] == goal_state) {
                        est_costs[ti] = 0.0f;
                    } else if(neighbor_exists[ti]) { // 1 + is due to the added cost from the neighbor
                        float cost = is_goal_state[ti] ? 0.0f : acc[ti];
                        est_costs[ti] = std::min(est_costs[ti], 1.0f + cost);
                    }
                }
            }
        }

        // at this point, we have found the minimum y_i values through 
        // value iteration from neighboring states, now train the network

        // encode the main puzzles & create an input tensor
        for(size_t i = 0; i < num_puzzles; ++i)
            one_hot_encode<uint8_t, float>(puzzles[i].get_tiles().data(), S,
                                           &encoded[i * Ssq]);
        auto inp_tensor = torch::from_blob(encoded.data(), 
                                           {static_cast<long>(num_puzzles), Ssq}).to(device);
        auto y_tensor = torch::from_blob(est_costs.data(), 
                                         {static_cast<long>(num_puzzles)}).to(device);

        // forward through the current network independently from the main one
        cur_optimizer.zero_grad();
        auto preds = cur_net->forward(inp_tensor);
        auto loss = torch::mse_loss(preds, y_tensor);
        loss.backward();
        cur_optimizer.step();

        float floss = loss.item<float>();

        size_t ndisplay = 8;
        if(step % print_every == 0) {
            std::cout << "Step: [" << step << '/' << num_iterations << "]\n";

            std::vector<int64_t> dists;
            for(size_t pi = 0; pi < ndisplay; ++pi) {
                auto result = search2::a_star_search<Puzzle, TSA, ManhH>(puzzles[pi]);
                dists.emplace_back(result.cost);
            }

            std::cout << "Distances: ";
            stream_itr(std::cout, dists.begin(), dists.begin()+ndisplay) << std::endl;

            std::cout << "Preds: ";
            stream_itr(std::cout, est_costs.begin(), est_costs.begin()+ndisplay) << std::endl;


            std::cout << "Loss: " << floss << std::endl;
        }

        // update main net if loss is less than the threshold
        if(floss < loss_threshold) {
            scramble_top = std::min(scramble_top+1, scramble_max);
            // std::cout << "Update done, scramble max: " << scramble_top << ".\n";
            torch::save(cur_net, TMP_NET_FILE_PATH);
            torch::load(main_net, TMP_NET_FILE_PATH);
        }
        // TODO: add checkpointing
        

    }



    return 0;
}
