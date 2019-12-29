#ifndef __SBPUZZLE_GENERATION_HPP__
#define __SBPUZZLE_GENERATION_HPP__

#include <cmath>
#include <vector>
#include <random>
#include <string>
#include <unordered_map>
#include <functional>

#include "sbpuzzle.hpp"

namespace sbpuzzle {

    enum class RandomGeneratorType {
        SHUFFLE,
        UNIFORM,
        SQRT
    };

    const std::vector<std::string> RANDOM_GENERATOR_STRINGS {
        "shuffle",
        "uniform",
        "sqrt"
    };

    RandomGeneratorType str2randomgeneratortype(const std::string &s) {
        static const std::unordered_map<std::string, RandomGeneratorType> map {
            {"shuffle", RandomGeneratorType::SHUFFLE},
            {"uniform", RandomGeneratorType::UNIFORM},
            {"sqrt", RandomGeneratorType::SQRT}
        };
        return map.at(s);
    }

    // a light wrapper around a discrete distribution that offsets it from (0, N)
    template <typename IntType> 
    class OffsetDiscreteDistribution {
    public:
        template <typename InputItr>
        OffsetDiscreteDistribution(InputItr wstart, InputItr wend, IntType off) 
            : distr(wstart, wend), offset(off) {}

        template <typename RNG>
        IntType operator()(RNG &rng) {
            return distr(rng) + offset;
        }

    private:
        std::discrete_distribution<IntType> distr;
        IntType offset;
    };

    // generate a weighted discrete distribution
    // weight by sqrt(scramble_max - scramble_min + 1)
    template <typename IntType, typename WeightF>
    OffsetDiscreteDistribution<IntType> make_offset_ddistr(IntType min, IntType max, WeightF f) {
        int64_t size = max - min + 1;
        double sum = 0.0;
        std::vector<double> weights(size);
        for(int64_t i = 0; i < size; ++i) 
            sum += (weights[i] = f(i + min));
        for(int64_t i = 0; i < size; ++i)
            weights[i] /= sum;
        return OffsetDiscreteDistribution<IntType>(weights.begin(), weights.end(), min);
    }


    // create puzzles by shuffling tiles randomly
    template <psize_t H, psize_t W, class URNG, class OutputIterator>
    void fill_shuffled_puzzles(size_t num_puzzles, URNG &rng, OutputIterator out) {
        std::array<uint8_t, H*W> tiles;
        std::iota(tiles.begin(), tiles.end(), 0);
        for(size_t i = 0; i < num_puzzles; ++i) {
            do {
                std::shuffle(tiles.begin(), tiles.end(), rng); 
            } while(!sbpuzzle::tiles_solvable<H, W>(tiles));
            *out++ = SBPuzzle<H, W>(tiles);
        }
    }

    template <psize_t H, psize_t W, class URNG, class OutputIterator, class Distribution>
    void fill_scrambled_puzzles(size_t num_puzzles,
                                URNG &rng,
                                OutputIterator puzzles_out,
                                Distribution &move_distribution)
    {
        // distribution for picking moves, lcm of 2, 3 and 4
        // much faster than making a single 0-11 and taking modulo
        static std::uniform_int_distribution<int64_t> ad2(0, 1);
        static std::uniform_int_distribution<int64_t> ad3(0, 2);
        static std::uniform_int_distribution<int64_t> ad4(0, 3);
        // fill a constant tile array with goal state values
        std::array<uint8_t, H*W> goal_tiles;
        std::iota(goal_tiles.begin(), goal_tiles.end(), 0);
        // std::cout << "MDM: " << move_distribution.max() << std::endl;
        for(size_t i = 0; i < num_puzzles; ++i) {
            // create a goal state puzzle and select a number of moves
            std::array<uint8_t, H*W> tiles = goal_tiles;
            int64_t num_moves = move_distribution(rng);
            // randomly apply a move to the puzzle num_moves times
            // first, generate num_moves valid moves
            uint8_t hp = details::HOLE<H, W>;
            uint8_t prev_action_inverse_index = -1;
            for(int64_t j = 0; j < num_moves; ++j) {
                const uint8_t *valid_actions = details::tiles_get_valid_moves<H, W>(hp);
                uint8_t num_valid = valid_actions[4];
                // select a random action index
                uint8_t random_index;
                switch(num_valid) {
                    case 2: random_index = ad2(rng); break;
                    case 3: random_index = ad3(rng); break;
                    case 4: random_index = ad4(rng); break;
                    default: throw std::runtime_error("Unexpected amount of valid moves");
                }
                // if the inverse action was previously done, select the next one instead
                random_index = (prev_action_inverse_index == valid_actions[random_index]) ?
                               random_index + 1 : random_index;
                // cmove is faster than modulo with a variable
                random_index = (random_index == num_valid) ? 0 : random_index;
                uint8_t action_index = valid_actions[random_index];
                // compute the tile position and apply the move
                uint8_t tp = hp + details::OFFSETS<W>[action_index];
                tiles[hp] = tiles[tp];
                hp = tp; // update hole position
                prev_action_inverse_index = details::inv_action_index(action_index);
            }
            tiles[hp] = details::HOLE<H, W>;
            // now that the action sequence is ready, apply it to the puzzle
            /*
            // debugging code
            for(auto a : action_sequence)
                std::cout << ((int)a.tpos) << "-" << ((int)a.hpos) << " ";
            std::cout << std::endl;
            std::cout << puzzle << std::endl;
            */
            // add the puzzle
            // if(num_moves_out != nullptr)
            //    (*num_moves_out)[i] = num_moves;
            *puzzles_out++ = SBPuzzle<H, W>(tiles);
        }
    }

    inline double unity(double x) { return 1.0; }
    inline double msqrt(double x) { return sqrt(x); }

    template <class URNG, class OutputIterator>
    using RandomGeneratorFn = void (*)(size_t, URNG &, OutputIterator);

    template <psize_t H, psize_t W, class URNG, class OutputIterator, class WeightF>
    std::function<void(size_t, URNG &, OutputIterator)> make_scrambled_generator(
            int64_t distr_min,
            int64_t distr_max,
            WeightF f)
    {
        return [=](size_t n, URNG &r, OutputIterator o) {
            auto distr = make_offset_ddistr(distr_min, distr_max, f);
            fill_scrambled_puzzles<H, W>(n, r, o, distr);
        };
    }

    template <psize_t H, psize_t W, class URNG, class OutputIterator>
    std::function<void(size_t, URNG &, OutputIterator)> random_sbpuzzle_generator_factory(
            RandomGeneratorType type, 
            int64_t distr_min=0,
            int64_t distr_max=0) 
    {
        switch(type) {
            case RandomGeneratorType::SHUFFLE:  
                return [](size_t n, URNG &r, OutputIterator o) {
                    fill_shuffled_puzzles<H, W>(n, r, o);
                };
            case RandomGeneratorType::UNIFORM:
                return make_scrambled_generator<H, W, URNG, OutputIterator>
                       (distr_min, distr_max, unity);
            case RandomGeneratorType::SQRT:
                return make_scrambled_generator<H, W, URNG, OutputIterator>
                       (distr_min, distr_max, sqrt); // from cmath
            default:
                throw std::invalid_argument("Unknown random generator type argument in factory.");
        }
    }

}

#endif
