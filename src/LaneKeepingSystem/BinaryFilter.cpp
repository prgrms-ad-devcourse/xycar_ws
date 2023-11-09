// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file BinaryFilter.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief PID Controller Class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include "LaneKeepingSystem/BinaryFilter.hpp"
namespace Xycar {

template <typename PREC>
BinaryFilter<PREC>::BinaryFilter(uint32_t sampleSize, PREC prior) : mSampleSize(sampleSize), mPrior(prior)
{
}

template <typename PREC>
void BinaryFilter<PREC>::addSample(bool newSample)
{
    mSamples.emplace_back(newSample);
    if (mSampleSize < static_cast<uint32_t>(mSamples.size()))
        mSamples.pop_front();

    mFilteringResult = update(static_cast<uint32_t>(mSamples.size()));
}

template <typename PREC>
PREC BinaryFilter<PREC>::update(uint32_t sampleSize)
{
    PREC success = 0.00001;
    PREC fail = 0.00001;

    for (uint32_t i = 0; i < sampleSize; ++i)
    {
        if (mSamples[i])
        {
            success += mPrior;
        }
        else
        {
            fail += 1.f - mPrior;
        }
    }
    PREC normalize = success + fail + 0.00001;
    return success / normalize;
}

template class BinaryFilter<float>;
template class BinaryFilter<double>;
} // namespace Xycar