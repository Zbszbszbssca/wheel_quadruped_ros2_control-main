//
// Created by biao on 24-9-16.
//
#include <cmath>
#include "rl_quadruped_controller/control/LowPassFilter.h"

/**
 * 低通滤波器，用于防止高频信号
 * @param samplePeriod 采样周期
 * @param cutFrequency 截止频率
 */
LowPassFilter::LowPassFilter(const double samplePeriod, const double cutFrequency)
{
    // 根据采样周期和截止频率计算滤波器的权重
    weight_ = 1.0 / (1.0 + 1.0 / (2.0 * M_PI * samplePeriod * cutFrequency));
    start_ = false;  // 初始化时尚未开始滤波
}

// 添加新值到滤波器中
void LowPassFilter::addValue(const double newValue)
{
    // 如果是第一次调用addValue函数，初始化pass_value_为新值
    if (!start_)
    {
        start_ = true;
        pass_value_ = newValue;
    }
    // 使用低通滤波公式更新pass_value_
    pass_value_ = weight_ * newValue + (1 - weight_) * pass_value_;
}

// 获取当前滤波器的输出值
double LowPassFilter::getValue() const
{
    return pass_value_;
}

// 清空滤波器的状态
void LowPassFilter::clear()
{
    start_ = false;  // 重置为未开始状态
}