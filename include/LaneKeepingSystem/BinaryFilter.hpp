// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file PIDController.hpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief PID Controller Class header file
 * @version 1.1
 * @date 2023-05-02
 */
#ifndef BINARY_FILTER_HPP_
#define BINARY_FILTER_HPP_

#include <deque>
#include <iostream>
#include <memory>
#include <vector>

namespace Xycar {
/**
 * @brief PID Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class BinaryFilter
{
public:
    using Ptr = std::unique_ptr<BinaryFilter>; ///< Pointer type of this class

    /**
     * @brief Construct a new BinaryFilter
     *
     * @param[in] pGain Proportional control gain
     * @param[in] iGain Integral control gain to remove error of steady-state
     * @param[in] dGain Differential control gain to relieve overshoot and improve stability
     */
    BinaryFilter(uint32_t sampleSize, PREC prior);

    /**
     * @brief Add new data to filter
     *
     * @param[in] newSample New position to be used in filtering
     */
    void addSample(bool newSample);

    /**
     * @brief Get the filtered data
     *
     * @return Result of weighted moving average filtering
     */
    const PREC getResult() const { return mFilteringResult; }

private:
    const PREC mPrior;
    const PREC mSampleSize;

    PREC mFilteringResult;

    std::deque<bool> mSamples; ///< Deque including values of samples

    PREC update(uint32_t sampleSize);
};
} // namespace Xycar
#endif // BINARY_FILTER_HPP_
