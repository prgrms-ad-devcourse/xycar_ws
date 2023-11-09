// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file PIDController.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief PID Controller Class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(int32_t errorFromMid)
{
    PREC castError = static_cast<PREC>(errorFromMid);
    mDifferentialGainError = castError - mProportionalGainError;
    mProportionalGainError = castError;
    mIntegralGainError += castError;
    return mProportionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError + mDifferentialGain * mDifferentialGainError;
}

template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar