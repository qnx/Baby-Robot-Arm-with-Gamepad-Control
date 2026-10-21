/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <cstdint>
#include <vector>

#include <spdlog/spdlog.h>

#include <hailo/hailort.hpp>

namespace joint::ml {
/**
 * @brief Wrapper for reading Hailo Tensor data meant to simply reading
 *        and handling output data.
 */
class OutputTensor {
   public:
    OutputTensor(hailort::OutputVStream &stream) : stream_(stream) {
        data_.resize(stream.get_frame_size());
        quantized_.resize(stream.get_frame_size());
    }

    inline size_t size() { return data_.size(); }
    inline void *data() { return data_.data(); }
    inline int read() { return stream_.read(hailort::MemoryView(data(), size())); }

    inline const std::vector<float> &quantized_data() { return quantized_; }
    inline std::string name() const { return stream_.name(); }
    const std::vector<float> &quantize() {
        auto quant_info = stream_.get_info().quant_info;

        spdlog::debug("quant min {} max {} scale {} zp {}", quant_info.limvals_min, quant_info.limvals_max,
                      quant_info.qp_scale, quant_info.qp_zp);

        for (int i = 0; i < data_.size(); i++) {
            switch (stream_.get_info().format.type) {
                case HAILO_FORMAT_TYPE_UINT8:
                    quantized_[i] = quant_info.qp_scale * (static_cast<int>(data_[i]) - quant_info.qp_zp);
                    break;
                case HAILO_FORMAT_TYPE_UINT16:
                    quantized_[i] = quant_info.qp_scale * (((uint16_t *)data_.data())[i] - quant_info.qp_zp);
                    break;
                case HAILO_FORMAT_TYPE_FLOAT32:
                    quantized_[i] = quant_info.qp_scale * (((float *)data_.data())[i] - quant_info.qp_zp);
                    break;
                default:
                    spdlog::critical("Unknown type");
                    abort();
            }
        }
        return quantized_;
    }

    size_t width() const { return stream_.get_info().shape.width; }

    size_t height() const { return stream_.get_info().shape.height; }

   private:
    std::vector<uint8_t> data_;
    std::vector<float> quantized_;
    hailort::OutputVStream &stream_;
};
}  // namespace joint::ml
