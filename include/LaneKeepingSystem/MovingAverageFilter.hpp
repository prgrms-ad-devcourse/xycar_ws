// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file MovingAverageFilter.hpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Moving Average Filter Class header file
 * @version 1.1
 * @date 2023-05-02
 */

#ifndef MOVING_AVERAGE_FILTER_HPP_
#define MOVING_AVERAGE_FILTER_HPP_

#include <deque>
#include <iostream>
#include <memory>
#include <vector>

namespace Xycar {

/**
 * @brief Moving Average Filtering Mode
 */
enum class FilteringMode : uint8_t
{
    NORMAL = 0,   ///< Normal average filtering
    WEIGHTED = 1, ///< Weighted average filtering
};

/**
 * @brief Moving Average Filter Class
 * @tparam PREC Precision of data
 * @tparam FilteringMode
 */
template <typename PREC, FilteringMode Mode = FilteringMode::WEIGHTED>
class MovingAverageFilter final
{
    // TODO : Implement abstract class like "Filtering" in order to prepare for extending filtering method
public:
    using Ptr = std::unique_ptr<MovingAverageFilter>; ///< Pointer type of this class

    /**
     * @brief Construct a new Moving Average Filter object
     *
     * @param[in] sampleSize Pixed sample size used
     */
    MovingAverageFilter(uint32_t sampleSize);

    /**
     * @brief Add new data to filter
     *
     * @param[in] newSample New position to be used in filtering
     */
    void addSample(int32_t newSample);

    /**
     * @brief Get the filtered data
     *
     * @return Result of weighted moving average filtering
     */
    const PREC getResult() const { return mFilteringResult; }

private:
    /**
     * @brief Filter samples every time a value is added
     *
     * @param[in] sampleSize Sample size to filer
     * @param[in] mode Filtering mode. Now moving average or Weighted moving average
     * @return The result of moving average filtering based on the mode of filtering
     */
    PREC update(uint32_t sampleSize);

    const uint32_t mSampleSize;    ///< Sample size to store samples
    std::deque<int32_t> mSamples;  ///< Deque including values of samples
    std::vector<int32_t> mWeights; ///< Weight values which increase from 1
    PREC mFilteringResult = 0.0;   ///< The result of filtering. This could be moved class 'Filtering' commented on TODO
};
} // namespace Xycar
#endif // MOVING_AVERAGE_FILTER_HPP_
