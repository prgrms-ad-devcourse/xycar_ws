// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file VehicleModel.hpp
 * @brief VehicleModel Class header file
 * @version 1.0
 * @date 2023-05-23
 */
#ifndef VEHICLE_MODEL_HPP_
#define VEHICLE_MODEL_HPP_

#include <iostream>
#include <cmath>
#include <tuple>
#include <memory>

namespace Xycar {
/**
 * @brief Motion model based on movement of vehicle
 * @tparam PREC Precision of data
 */
template <typename PREC>
class VehicleModel
{
public:
    using Ptr = std::unique_ptr<VehicleModel>;
    /**
     *  이 코드는 예제로서, 알맞은 방법으로 자유롭게 수정하셔야 합니다
     */

    static constexpr PREC wheelBase = 0.325; ///<

    /**
     * @brief Construct a new Vehicle object
     *
     * @param[in] initialX
     * @param[in] initialY
     * @param[in] initialHeading
     */
    VehicleModel(PREC initialX, PREC initialY, PREC initialHeading) : mX(initialX), mY(initialY), mHeading(initialHeading) {}

    /**
     * @brief Update the vehicle's state based on the inputs
     *
     * @param[in] velocity
     * @param[in] deltaSteeringAngle
     * @param[in] dt
     */
    void update(PREC velocity, PREC deltaSteeringAngle, PREC dt);

    /**
     * @brief Get the result of motion model
     *
     * @return The result of motion model
     */
    const std::tuple<PREC, PREC, PREC> getResult() const {
        return std::make_tuple(mX, mY, mHeading);
    };

private:
    PREC mX;       // x-coordinate of the vehicle
    PREC mY;       // y-coordinate of the vehicle
    PREC mHeading; // Heading angle of the vehicle. Yaw
};

} // namespace Xycar
#endif // VEHICLE_MODEL_HPP_