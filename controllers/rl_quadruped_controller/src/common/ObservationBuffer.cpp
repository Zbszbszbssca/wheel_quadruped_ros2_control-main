//
// Created by biao on 24-10-6.
//

#include "ObservationBuffer.h"

// 构造函数，初始化ObservationBuffer
ObservationBuffer::ObservationBuffer(int num_envs,
                                     const int num_obs,
                                     const int include_history_steps)
    : num_envs_(num_envs),
      num_obs_(num_obs),
      include_history_steps_(include_history_steps) {
    // 计算总的观测数，并初始化观测缓冲区为零
    num_obs_total_ = num_obs_ * include_history_steps_;
    obs_buffer_ = torch::zeros({num_envs_, num_obs_total_}, dtype(torch::kFloat32));
}

// 重置特定环境的观测值
void ObservationBuffer::reset(const std::vector<int> &reset_index, const torch::Tensor &new_obs) {
    std::vector<torch::indexing::TensorIndex> indices;
    // 为每个需要重置的环境添加索引
    for (int index: reset_index) {
        indices.emplace_back(torch::indexing::Slice(index));
    }
    // 重置特定环境的观测值，重复观测值以填充历史步骤
    obs_buffer_.index_put_(indices, new_obs.repeat({1, include_history_steps_}));
}

// 清空观测缓冲区
void ObservationBuffer::clear()
{
    obs_buffer_ = torch::zeros_like(obs_buffer_);
}

// 用当前观测复制 N 次，填满所有历史帧（ IsaacLab 核心）
void ObservationBuffer::resetAll(const torch::Tensor& new_obs)
{
    obs_buffer_ = new_obs.repeat({1, include_history_steps_});
}


// 将新观测值插入到缓冲区中
void ObservationBuffer::insert(const torch::Tensor &new_obs) {
    // 将现有的观测值向后移动
    const torch::Tensor shifted_obs = obs_buffer_.index({
        torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(num_obs_, num_obs_ * include_history_steps_)
    }).clone();
    
    // 更新缓冲区，填充新的历史观测
    obs_buffer_.index({
        torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(0, num_obs_ * (include_history_steps_ - 1))
    }) = shifted_obs;

    // 将新的观测值添加到缓冲区
    obs_buffer_.index({
        torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(-num_obs_, torch::indexing::None)
    }) = new_obs;
}

// 获取指定观测ID的历史观测值
torch::Tensor ObservationBuffer::getObsVec(const std::vector<int> &obs_ids) const {
    std::vector<torch::Tensor> obs;
    // 遍历观测ID，从最新的历史观测开始
    for (int i = obs_ids.size() - 1; i >= 0; --i) {
        const int obs_id = obs_ids[i];
        const int slice_idx = include_history_steps_ - obs_id - 1;
        // 提取特定时间步的观测数据
        obs.push_back(obs_buffer_.index({
            torch::indexing::Slice(torch::indexing::None),
            torch::indexing::Slice(slice_idx * num_obs_, (slice_idx + 1) * num_obs_)
        }));
    }
    // 将所有提取的观测数据连接在一起返回
    return cat(obs, -1);
}