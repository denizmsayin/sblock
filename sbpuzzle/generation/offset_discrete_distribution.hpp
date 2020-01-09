#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_GENERATION_OFFSET_DISCRETE_DISTRIBUTION_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_GENERATION_OFFSET_DISCRETE_DISTRIBUTION_HPP

#include <random>

namespace denizmsayin::sblock::sbpuzzle::generation {
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
}

#endif
