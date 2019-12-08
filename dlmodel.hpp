#ifndef __DLMODEL_HPP__
#define __DLMODEL_HPP__

#include <torch/script.h>

#include <cmath>
#include <cstdint>
#include <array>
#include <memory>

#include "sbpuzzle.hpp"

namespace sbpuzzle {

    namespace details {
        template <size_t S, typename T>
        void one_hot_encode(const std::array<uint8_t, S> &inp, std::array<T, S*S> &out) {
            out.fill(static_cast<T>(0));
            for(size_t i = 0; i < S; ++i)
                out[i * S + inp[i]] = static_cast<T>(1);
        }
    }

    template <size_t S>
    class DLModel {
    public:
        virtual ~DLModel() {}

        uint8_t forward(const std::array<uint8_t, S> &tiles) {
            // one-hot encode the tiles to float32
            std::array<float, S*S> enc;
            details::one_hot_encode<S>(tiles, enc);

            // create an input tensor
            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(torch::from_blob(&enc[0], {1, S*S}).to(device));
            
            // return the output tensor
            at::Tensor output = module.forward(inputs).toTensor();
            return convert_output_tensor(output);
        }   

        uint8_t forward(const SBPuzzleWHole<H, W> &p) {
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
        }

        DLModel(const std::string &filename, const torch::Device &dev) : device(dev) 
        {
            module = torch::jit::load(filename);
        }

        virtual uint8_t convert_output_tensor(const at::Tensor &output) const = 0;
    };

    namespace details {
        template <size_t S>
        class RegressionModel : public DLModel<S> {
        public:
            RegressionModel(torch::jit::script::Module &&m, const torch::Device &dev) 
                : DLModel<S>(std::move(m), dev) {}

            uint8_t convert_output_tensor(const at::Tensor &output) const {
                return static_cast<uint8_t>(round(output.item().toFloat()));
            }
        };
        
        template <size_t S>
        class ClassificationModel : public DLModel<S> {
        public:
            ClassificationModel(torch::jit::script::Module &&m, const torch::Device &dev) 
                : DLModel<S>(std::move(m), dev) {}

            uint8_t convert_output_tensor(const at::Tensor &output) const {
                return static_cast<uint8_t>(at::argmax(output).item().toInt());
            }
        };
    }

    template <size_t S>
    std::unique_ptr<DLModel<S>> DLModel<S>::from_file(const std::string &file_path, 
                                                      const torch::Device &dev)
    {
        using details::RegressionModel;
        using details::ClassificationModel;
        torch::jit::script::Module m = torch::jit::load(file_path);
        // pass an empty tensor through the module to determine the output size
        std::vector<torch::jit::IValue> dummy_inputs {torch::empty({1, S*S})};
        auto dummy_out = m.forward(dummy_inputs).toTensor();
        int64_t out_size = at::size(dummy_out, -1);
        if(out_size == 1)
            return std::unique_ptr<DLModel<S>>(new RegressionModel<S>(std::move(m), dev));
        else
            return std::unique_ptr<DLModel<S>>(new ClassificationModel<S>(std::move(m), dev));
    }

}


#endif
