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

#include "LaneKeepingSystem/LaneKalmanFilter.hpp"
namespace Xycar {

template <typename PREC>
void LaneKalmanFilter<PREC>::predict(const Eigen::Vector2d& u)
{
    mX = mF * mX + mB * u;
    mP = mF * mP * mF.transpose() + mQ;
}

template <typename PREC>
void LaneKalmanFilter<PREC>::update(const Eigen::Vector2d& z)
{
    Eigen::Vector2d y = z - mH * mX;
    Eigen::Matrix2d S = mH * mP * mH.transpose() + mR;
    Eigen::Matrix<double, 2, 2> K = mP * mH.transpose() * S.inverse();

    mX = mX + K * y;
    mP = (Eigen::Matrix2d::Identity() - K * mH) * mP;
}

template class LaneKalmanFilter<float>;
template class LaneKalmanFilter<double>;
} // namespace Xycar