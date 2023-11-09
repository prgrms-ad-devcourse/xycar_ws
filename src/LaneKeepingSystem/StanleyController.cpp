// Copyright (C) 2023 Grepp CO.
// All rights reserved.

#include "LaneKeepingSystem/StanleyController.hpp"

namespace Xycar {

template <typename PREC>
void StanleyController<PREC>::calculateSteeringAngle(PREC crossTrackError, PREC headingError, PREC velocity)
{
    // Calculate the cross-track error (cte) compensation
    PREC alpha = std::atan2(mGain * crossTrackError, velocity);

    // Calculate the desired heading angle
    PREC desiredHeading = this->normalizeAngle(headingError) + alpha;

    // Calculate the steering angle using the desired heading and look-ahead distance
    // PREC steeringAngle = std::atan2(2 * mLookAheadDistance * std::sin(desiredHeading), velocity);

    mResult = desiredHeading * (180.f / this->getPI());
}

template class StanleyController<float>;
template class StanleyController<double>;
} // namespace Xycar
