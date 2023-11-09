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
#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cstdint>
#include <memory>

namespace Xycar {
/**
 * @brief PID Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class PIDController
{
public:
    using Ptr = std::unique_ptr<PIDController>; ///< Pointer type of this class

    /**
     * @brief Construct a new PID object
     *
     * @param[in] pGain Proportional control gain
     * @param[in] iGain Integral control gain to remove error of steady-state
     * @param[in] dGain Differential control gain to relieve overshoot and improve stability
     */
    PIDController(PREC pGain, PREC iGain, PREC dGain) : mProportionalGain(pGain), mIntegralGain(iGain), mDifferentialGain(dGain) {}

    /**
     * @brief Compute and return the PID Control Output
     *
     * @param[in] errorFromMid Error between estimated position x and half of the image
     * @return The result of PID controller
     */
    PREC getControlOutput(int32_t errorFromMid);

private:
    const PREC mProportionalGain;      ///< Proportional control gain. The higher, the more powerful
    const PREC mIntegralGain;          ///< Integral control gain to remove error of steady-state
    const PREC mDifferentialGain;      ///< Differential control gain to relieve overshoot and improve stability
    PREC mProportionalGainError = 0.0; ///< Error to determine how much the proportional gain should be reflected
    PREC mIntegralGainError = 0.0;     ///< Error to determine how much the integral gain should be reflected
    PREC mDifferentialGainError = 0.0; ///< Error to determine how much the differential gain should be reflected
};
} // namespace Xycar
#endif // PID_CONTROLLER_HPP_
