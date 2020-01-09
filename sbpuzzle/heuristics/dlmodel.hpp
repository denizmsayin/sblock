#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_HEURISTICS_DLMODEL_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_HEURISTICS_DLMODEL_HPP

#ifdef W_TORCH

#include <torch/script.h>

#include <cmath>
#include <cstdint>
#include <array>
#include <memory>

#include "../basic.hpp"

namespace denizmsayin::sblock::sbpuzzle::heuristics {

    namespace details {
        template <typename T1, typename T2>
        void one_hot_encode(const T1 *inp, size_t s, T2 *out) {
            std::fill(out, out + s*s, static_cast<T2>(0));
            for(size_t i = 0; i < s; ++i)
                out[i * s + static_cast<size_t>(inp[i])] = static_cast<T2>(1);
        }
    }

    template <psize_t H, psize_t W>
    class DLModel {
    public:
        virtual ~DLModel() {}

        pcost_t forward(const std::array<pcell_t, H*W> &tiles) {
            static constexpr size_t S = sbpuzzle::details::SIZE<H, W>;
            // one-hot encode the tiles to float32
            std::array<float, S*S> enc;
            details::one_hot_encode<pcell_t, float>(tiles.data(), S, enc.data());

            // create an input tensor
            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(torch::from_blob(&enc[0], {1, S*S}).to(device));
            
            // return the output tensor
            at::Tensor output = module.forward(inputs).toTensor();
            return convert_output_tensor(output);
        }   

        template <typename IType, typename OType,
                  typename RandomAccessIterator, typename OutputIterator>
        void forward(RandomAccessIterator begin,
                     RandomAccessIterator end,
                     OutputIterator out)
        {
            static constexpr size_t S = sbpuzzle::details::SIZE<H, W>;
            // one hot encode all the puzzles in a cont. memory block
            long int n = static_cast<long int>(end - begin);
            std::vector<IType> enc(n * S * S); // in cont. memory
            size_t i = 0;
            for(auto itr = begin; itr != end; ++itr) {
                details::one_hot_encode<pcell_t, IType>(itr->get_tiles().data(), S, &enc[i]);
                i += S * S;
            }

            // push them back and forward them, output is {n, output_dim}
            std::vector<torch::jit::IValue> inputs;
            auto input_tensor = torch::from_blob(enc.data(), {n, S*S}).to(device);
            inputs.push_back(input_tensor);
            auto output_tensor = module.forward(inputs).toTensor();

            // convert the output tensor to a {n,} result tensor, depends on model type
            auto result_tensor = output2result(output_tensor).to(torch::kCPU);

            // empty the result tensor to output itr
            auto acc = result_tensor.template accessor<long, 1>();
            for(long int i = 0; i < n; ++i)
                *out++ = static_cast<OType>(acc[i]);
        }

        uint8_t forward(const Basic<H, W> &p) {
            return forward(p.get_tiles());
        }

        static std::unique_ptr<DLModel> from_file(const std::string &filename, 
                                                  const torch::Device &dev);

    protected:
        torch::jit::script::Module module;
        torch::Device device;
        
        DLModel(torch::jit::script::Module &&m, const torch::Device &dev) 
            : module(m), device(dev)
        {
            module.to(device);
            module.eval();
        }

        DLModel(const std::string &filename, const torch::Device &dev) 
            : DLModel(torch::jit::load(filename), dev) {}

        virtual pcost_t convert_output_tensor(const at::Tensor &output) const = 0;
        virtual at::Tensor output2result(const at::Tensor &output) const = 0;
    };

    namespace details {
        template <psize_t H, psize_t W>
        class RegressionModel : public DLModel<H, W> {
        public:
            RegressionModel(torch::jit::script::Module &&m, const torch::Device &dev) 
                : DLModel<H, W>(std::move(m), dev) {}

            pcost_t convert_output_tensor(const at::Tensor &output) const {
                return static_cast<pcost_t>(round(output.item().toFloat()));
            }
        
            at::Tensor output2result(const at::Tensor &output) const {
                // input: {n, 1} regression results, output {n,} integral costs
                return at::round(at::squeeze(output, 1)).to(at::kLong);
            }
        };
        
        template <psize_t H, psize_t W>
        class ClassificationModel : public DLModel<H, W> {
        public:
            ClassificationModel(torch::jit::script::Module &&m, const torch::Device &dev) 
                : DLModel<H, W>(std::move(m), dev) {}

            pcost_t convert_output_tensor(const at::Tensor &output) const {
                return static_cast<pcost_t>(at::argmax(output).item().toInt());
            }

            at::Tensor output2result(const at::Tensor &output) const {
                // input: {n, maxcost} classification results, output {n,} integral costs
                return at::argmax(output, 1);
            }
        };
    }

    template <psize_t H, psize_t W>
    std::unique_ptr<DLModel<H, W>> DLModel<H, W>::from_file(const std::string &file_path, 
                                                            const torch::Device &dev)
    {
        using details::RegressionModel;
        using details::ClassificationModel;
        torch::jit::script::Module m = torch::jit::load(file_path);
        m.to(dev);
        // pass an empty tensor through the module to determine the output size
        std::vector<torch::jit::IValue> dummy_inputs {torch::empty({1, H*H*W*W}).to(dev)};
        auto dummy_out = m.forward(dummy_inputs).toTensor();
        int64_t out_size = at::size(dummy_out, -1);
        if(out_size == 1)
            return std::unique_ptr<DLModel<H, W>>(new RegressionModel<H, W>(std::move(m), dev));
        else
            return std::unique_ptr<DLModel<H, W>>(new ClassificationModel<H, W>(std::move(m), dev));
    }

}


#endif

#endif
