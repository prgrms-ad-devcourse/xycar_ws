// Copyright (C) 2023 Grepp CO.
// All rights reserved.

#include "LaneKeepingSystem/VehicleModel.hpp"

namespace Xycar {

template <typename PREC>
void VehicleModel<PREC>::update(PREC velocity, PREC deltaSteeringAngle, PREC dt)
{
    // Update the position
    mX += velocity * std::cos(mHeading) * dt;
    mY += velocity * std::sin(mHeading) * dt;

    // Update the heading
    mHeading += velocity / wheelBase * std::tan(deltaSteeringAngle) * dt;
}

template class VehicleModel<float>;
template class VehicleModel<double>;
} // namespace Xycar