// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file Controller.hpp
 * @brief Controller Class header file
 * @version 1.0
 * @date 2023-05-23
 */
#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <cmath>
#include <cstdint>
#include <memory>

namespace Xycar {
/**
 * @brief Absbract Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class Controller
{
public:
    using Ptr = std::unique_ptr<Controller>; ///< Pointer type of this class

    /**
     * @brief Construct a new Controller object
     */
    Controller() = default;

    /**
     * @brief Return the Control Output
     *
     * @return The result of controller
     */
    PREC getControlOutput() const { return mResult; }

    /**
     * @brief
     *
     * @param angle
     * @return
     */
    PREC normalizeAngle(PREC angle)
    {
        const PREC pi = getPI();
        while (angle > pi)
            angle -= 2 * pi;
        while (angle < -pi)
            angle += 2 * pi;
        return angle;
    }

protected:
    PREC mResult; ///< Control output
    /**
     * @brief
     *
     * @return constexpr double
     */
    constexpr PREC getPI() { return static_cast<PREC>(std::atan(1) * 4.0); }
};
} // namespace Xycar
#endif // CONTROLLER_HPP_