// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file main.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Chihyeon Lee
 * @brief Lane Keeping System Main Function using Hough Transform
 * @version 1.1
 * @date 2023-05-02
 */
#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

using PREC = float;

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "Lane Keeping System");
    Xycar::LaneKeepingSystem<PREC> laneKeepingSystem;
    laneKeepingSystem.run();

    return 0;
}
