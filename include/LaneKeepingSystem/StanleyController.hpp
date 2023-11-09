// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file StanleyController.hpp
 * @brief StanleyController Class header file
 * @version 1.0
 * @date 2023-05-23
 */
#ifndef STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER_HPP_

#include "Controller.hpp"
#include <cmath>
#include <cstdint>
#include <memory>

namespace Xycar {
/**
 * @brief Stanley Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class StanleyController : public Controller<PREC>
{
public:
    using Ptr = std::unique_ptr<StanleyController>; ///< Pointer type of this class
    /**
     *  이 코드는 예제로서, 알맞은 방법으로 자유롭게 수정하셔야 합니다
     */

    /**
     *
     * @brief Construct a new Stanley Controller object
     *
     * @param[in] gain Stanley control gain
     * @param[in] lookAheadDistance Look-ahead distance
     */
    StanleyController(PREC gain, PREC lookAheadDistance) : mGain(gain), mLookAheadDistance(lookAheadDistance) {}

    /**
     * @brief
     *
     * @param[in] crossTrackError
     * @param[in] headingError
     * @param[in] velocity
     * @return
     */
    void calculateSteeringAngle(PREC crossTrackError, PREC headingError, PREC velocity);

    double getResult() { return mResult; };

private:
    double mGain;              ///< Stanley control gain
    double mLookAheadDistance; ///< Look-ahead distance
    double mResult;
};
} // namespace Xycar
#endif // STANLEY_CONTROLLER_HPP_
