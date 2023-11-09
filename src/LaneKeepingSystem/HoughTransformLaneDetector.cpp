// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file HoughTransformLaneDetector.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief hough transform lane detector class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include <numeric>

#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"

namespace Xycar {

template <typename PREC>
void HoughTransformLaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();
    mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();
    mHoughLineSlopeRange = config["HOUGH"]["ABS_SLOPE_RANGE"].as<PREC>();
    mHoughThreshold = config["HOUGH"]["THRESHOLD"].as<int32_t>();
    mHoughMinLineLength = config["HOUGH"]["MIN_LINE_LENGTH"].as<int32_t>();
    mHoughMaxLineGap = config["HOUGH"]["MAX_LINE_GAP"].as<int32_t>();
    mDebugging = config["DEBUG"].as<bool>();
    mContinusThreshold = config["HOUGH"]["CONTINUS_THRESHOLD"].as<int32_t>();
    mHoughClusterDistance = config["HOUGH"]["CLUSTER_DISTANCE"].as<PREC>();
    mHoughRadian = config["HOUGH"]["CLUSTER_RADIAN"].as<PREC>();

    mPrevLeftPosition = 220;
    mPrevRightPosition = 420;

    Eigen::Vector2d initialLeftX;
    Eigen::Vector2d initialRightX;
    Eigen::Matrix2d kalmanP;
    Eigen::Matrix2d kalmanF;
    Eigen::Matrix2d kalmanH;
    Eigen::Matrix2d kalmanQ;
    Eigen::Matrix2d kalmanR;
    Eigen::Matrix2d kalmanB;

    initialLeftX << 220, -0.1;
    initialRightX << 420, 0.1;
    kalmanP << 0.7, 0, 0, 0.3;
    kalmanF << 1, 1, 0, 0;
    kalmanH << 1, 1, 0, 0;
    kalmanQ << 0.025, 0, 0, 0.025;
    kalmanR << 0.1, 0, 0, 0.1;
    kalmanB << 1, 0, 0, 1;

    mLeftKalmanFilter = std::make_unique<LaneKalmanFilter<PREC>>(initialLeftX, kalmanP, kalmanF, kalmanH, kalmanQ, kalmanR, kalmanB);
    mRightKalmanFilter = std::make_unique<LaneKalmanFilter<PREC>>(initialRightX, kalmanP, kalmanF, kalmanH, kalmanQ, kalmanR, kalmanB);
}

template <typename PREC>
std::pair<PREC, PREC> HoughTransformLaneDetector<PREC>::getLineParameters(const Lines& lines, const Indices& lineIndices)
{
    uint32_t numLines = static_cast<uint32_t>(lineIndices.size());
    if (numLines == 0)
        return { 0.0f, 0.0f };

    int32_t xSum = 0;
    int32_t ySum = 0;
    PREC mSum = 0.0f;
    for (const auto lineIndex : lineIndices)
    {
        int32_t x1 = lines[lineIndex][HoughIndex::x1];
        int32_t y1 = lines[lineIndex][HoughIndex::y1];
        int32_t x2 = lines[lineIndex][HoughIndex::x2];
        int32_t y2 = lines[lineIndex][HoughIndex::y2];
        xSum += x1 + x2;
        ySum += y1 + y2;
        mSum += static_cast<PREC>((y2 - y1)) / (x2 - x1);
    }

    PREC xAverage = static_cast<PREC>(xSum) / (numLines * 2);
    PREC yAverage = static_cast<PREC>(ySum) / (numLines * 2);
    PREC m = mSum / numLines;
    PREC b = yAverage - m * xAverage;

    return { m, b };
}

template <typename PREC>
int32_t HoughTransformLaneDetector<PREC>::getLinePositionX(const Lines& lines, const Indices& lineIndices, Direction direction)
{
    const auto [m, b] = getLineParameters(lines, lineIndices);

    if (std::abs(m) <= std::numeric_limits<PREC>::epsilon() && std::abs(b) <= std::numeric_limits<PREC>::epsilon())
    {
        if (direction == Direction::LEFT)
            return 0.0f;
        else if (direction == Direction::RIGHT)
            return static_cast<PREC>(mImageWidth);
    }

    PREC y = static_cast<PREC>(mROIHeight) * 0.5f;
    return std::round((y - b) / m);
}

template <typename PREC>
std::pair<std::vector<std::vector<int>>, bool> HoughTransformLaneDetector<PREC>::divideLines(const Lines& lines)
{
    std::vector<Indices> clusterLineIndices;
    Indices leftLineIndices;
    Indices rightLineIndices;
    uint32_t linesSize = static_cast<uint32_t>(lines.size());
    leftLineIndices.reserve(linesSize);
    rightLineIndices.reserve(linesSize);
    PREC slope = 0.0f;
    PREC leftLineSumX = 0.0f;
    PREC rightLineSumX = 0.0f;

    std::vector<std::array<double, 7>> lanes;

    for (uint32_t i = 0; i < linesSize; ++i)
    {
        const auto& line = lines[i];

        double x1 = static_cast<double>(line[HoughIndex::x1]);
        double y1 = static_cast<double>(line[HoughIndex::y1]);
        double x2 = static_cast<double>(line[HoughIndex::x2]);
        double y2 = static_cast<double>(line[HoughIndex::y2]);

        double centerX = (x1 + x2) / 2;
        double centerY = (y1 + y2) / 2;

        double radian = std::atan2(std::abs(y1 - y2), x1 - x2);
        std::array<double, 7> laneInfo = { x1, y1, x2, y2, centerX, centerY, radian };

        lanes.push_back(laneInfo);
    }

    linesSize = static_cast<uint32_t>(lanes.size());

    std::vector<bool> visited(linesSize, false);
    std::vector<std::vector<int32_t>> clusterLaneIndexs;
    const PREC distanceThreshold = mHoughClusterDistance;
    const PREC radianThreshold = mHoughRadian;   // threshold raidan 30 degree -> radian
    const PREC radianLowerThreshold = 0.0872665; // 10 degree to radian
    const PREC radianUpperThreshold = 3.05433;   // 170 degree to radian
    bool hasStopLane = false;
    PREC centerX = (mPrevLeftPosition + mPrevRightPosition) / 2;

    for (uint32_t i = 0; i < linesSize; i++)
    {
        auto currentLane = lanes[i];
        std::vector<int> cluster;
        std::queue<int> queue;

        if (visited[i])
        {
            continue;
        }
        //  || currentLane[6] <= radianLowerThreshold || currentLane[6] >= radianUpperThreshold

        visited[i] = true;
        queue.push(i);

        while (!queue.empty())
        {
            int currentIndex = queue.front();
            auto currentLane = lanes[currentIndex];
            queue.pop();
            cluster.push_back(currentIndex);

            for (uint32_t j = 0; j < linesSize; j++)
            {
                if (visited[j])
                {
                    continue;
                }

                auto targetLane = lanes[j];
                PREC distance = mCalculateDistance(currentLane[4], currentLane[5], targetLane[4], targetLane[5]);
                double radian = std::atan2(std::abs(currentLane[4] - targetLane[4]), std::abs(currentLane[5] - targetLane[5]));
                double radianDiff = std::abs(currentLane[6] - targetLane[6]);

                if (radianDiff <= radianThreshold && distance <= distanceThreshold)
                {
                    queue.push(j);
                    visited[j] = true;
                }
            }
        }
        if (cluster.size() > 2)
        {
            clusterLaneIndexs.push_back(cluster);
            int32_t clusterSize = cluster.size();
            PREC sumCenterX = 0.f;
            PREC sumRadian = 0.f;
            PREC averageCenterX = 0.f;
            PREC averageRadian = 0.f;

            for (auto laneIndex : cluster)
            {
                auto lane = lanes[i];
                sumCenterX += lane[4];
                sumRadian += lane[6];
            }

            averageCenterX = sumCenterX / static_cast<PREC>(clusterSize);
            averageRadian = sumRadian / static_cast<PREC>(clusterSize);

            // std::cout << "center x: " << averageCenterX << ", radian: " << averageRadian << std::endl;

            if (std::abs(centerX - averageCenterX) <= static_cast<PREC>(mImageWidth) * 0.05 && (averageRadian >= radianUpperThreshold || averageRadian <= radianLowerThreshold))
            {
                hasStopLane = true;
            }
        }
    }

    return { clusterLaneIndexs, hasStopLane };
}

template <typename PREC>
std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::predictLanePosition(Eigen::Vector2d& inputVector)
{
    mLeftKalmanFilter->predict(inputVector);
    mRightKalmanFilter->predict(inputVector);

    Eigen::Vector2d leftResult = mLeftKalmanFilter->getState();
    Eigen::Vector2d rightResult = mRightKalmanFilter->getState();

    mPrevLeftPosition = leftResult(0);
    mPrevRightPosition = rightResult(0);

    return { leftResult(0), rightResult(0) };
}

template <typename PREC>
std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::getLanePosition(const cv::Mat& image, bool runUpdate, std::string detection)
{
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    cv::Mat bilateralFilterImage;
    cv::bilateralFilter(grayImage, bilateralFilterImage, 9, 75, 75);

    cv::Mat cannyImage;
    cv::Canny(bilateralFilterImage, cannyImage, mCannyEdgeLowThreshold, mCannyEdgeHighThreshold);

    cv::Mat ROI = cannyImage(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));
    Lines allLines;
    cv::HoughLinesP(ROI, allLines, kHoughRho, kHoughTheta, mHoughThreshold, mHoughMinLineLength, mHoughMaxLineGap);

    if (allLines.empty())
        return { mPrevLeftPosition, mPrevRightPosition };

    const auto [lanes, detectedStopLane] = divideLines(allLines);
    std::vector<PREC> lanePositions;

    for (int i = 0; i < lanes.size(); i++)
    {
        auto lanePositionX = getLinePositionX(allLines, lanes[i], Direction::NONE);
        bool overlap = false;
        if (lanePositionX >= mImageWidth || lanePositionX <= 0)
        {
            continue;
        }
        for (int j = i + 1; j < lanes.size(); j++)
        {
            auto targetLanePositionX = getLinePositionX(allLines, lanes[j], Direction::NONE);
            if (std::abs(lanePositionX - targetLanePositionX) <= 100)
            {
                overlap = true;
                break;
            }
        }

        if (!overlap)
        {
            lanePositions.push_back(static_cast<PREC>(lanePositionX));
        }
    }

    int i = 0;
    PREC left_minimum_distance = mPrevLeftPosition == -1 ? 10000 : mContinusThreshold;
    PREC right_minimum_distance = mPrevRightPosition == -1 ? 10000 : mContinusThreshold;

    auto leftPosition = mPrevLeftPosition;
    auto rightPosition = mPrevRightPosition;

    bool leftUpdated = false;
    bool rightUpdated = false;

    if (lanePositions.size() == 1)
    {
        auto leftGap = std::abs(mPrevLeftPosition - lanePositions[0]);
        auto rightGap = std::abs(mPrevRightPosition - lanePositions[0]);

        // std::cout << leftGap << ", " << rightGap << std::endl;
        if (leftGap < rightGap)
        {
            leftUpdated = true;
            leftPosition = lanePositions[0];
        }
        else
        {
            rightUpdated = true;
            rightPosition = lanePositions[0];
        }
    }
    else
    {
        for (auto lanePositionX : lanePositions)
        {
            cv::rectangle(mDebugFrame, cv::Point(lanePositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                          cv::Point(lanePositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kPurple, kDebugLineWidth);
            auto leftGap = std::abs(mPrevLeftPosition - lanePositionX);
            auto rightGap = std::abs(mPrevRightPosition - lanePositionX);

            if (leftGap < rightGap && leftGap < left_minimum_distance)
            {
                leftUpdated = true;
                leftPosition = lanePositionX;
                left_minimum_distance = leftGap;
            }
            else if (rightGap < leftGap && rightGap < right_minimum_distance)
            {
                rightUpdated = true;
                rightPosition = lanePositionX;
                right_minimum_distance = rightGap;
            }
        }
    }

    if (detection == "LEFT") {
        rightPosition =  leftPosition + 200;
    }
    if (detection == "RIGHT") {
        leftPosition =  rightPosition - 200;
    }

    if (leftPosition > rightPosition)
    {
        std::swap(leftPosition, rightPosition);
    }

    if (leftUpdated && runUpdate)
    {
        Eigen::Vector2d measureVector;
        PREC gradient = leftPosition - mPrevLeftPosition;
        measureVector << leftPosition, gradient;
        mLeftKalmanFilter->update(measureVector);
    }

    if (rightUpdated && runUpdate)
    {
        Eigen::Vector2d measureVector;
        PREC gradient = rightPosition - mPrevRightPosition;
        measureVector << rightPosition, gradient;
        mRightKalmanFilter->update(measureVector);
    }

    Eigen::Vector2d leftResult = mLeftKalmanFilter->getState();
    Eigen::Vector2d rightResult = mRightKalmanFilter->getState();

    // std::cout << "left: " << leftResult(0) << ", " << leftResult(1) << std::endl;
    // std::cout << "right: " << rightResult(0) << ", " << rightResult(1) << std::endl;

    mPrevLeftPosition = leftResult(0);
    mPrevRightPosition = rightResult(0);
    mStopDetected = detectedStopLane;

    return { leftResult(0), rightResult(0) };
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawLines(const Lines& lines, const Indices& leftLineIndices, const Indices& rightLineIndices)
{
    auto draw = [this](const Lines& lines, const Indices& indices) {
        for (const auto index : indices)
        {
            const auto& line = lines[index];
            auto r = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto g = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto b = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();

            cv::line(mDebugFrame, { line[static_cast<uint8_t>(HoughIndex::x1)], line[static_cast<uint8_t>(HoughIndex::y1)] + mROIStartHeight },
                     { line[static_cast<uint8_t>(HoughIndex::x2)], line[static_cast<uint8_t>(HoughIndex::y2)] + mROIStartHeight }, { b, g, r }, kDebugLineWidth);
        }
    };

    draw(lines, leftLineIndices);
    draw(lines, rightLineIndices);
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawRectangles(int32_t leftPositionX, int32_t rightPositionX, int32_t estimatedPositionX)
{
    cv::rectangle(mDebugFrame, cv::Point(leftPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(leftPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(rightPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(rightPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(estimatedPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(estimatedPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kRed, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(mImageWidth / 2 - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(mImageWidth / 2 + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kBlue, kDebugLineWidth);
}

template <typename PREC>
PREC HoughTransformLaneDetector<PREC>::mCalculateDistance(PREC x1, PREC y1, PREC x2, PREC y2)
{
    PREC dx = x1 - x2;
    PREC dy = y1 - y2;

    return std::sqrt(dx * dx + dy * dy);
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::setPrevLinePosition(const int32_t leftPosition, const int32_t rightPosition)
{
    mPrevLeftPosition = leftPosition;
    mPrevRightPosition = rightPosition;
}

template <typename PREC>
std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::getPrevLinePosition()
{
    return { mPrevLeftPosition, mPrevRightPosition };
}

template <typename PREC>
bool HoughTransformLaneDetector<PREC>::getStopLineStatus()
{
    return mStopDetected;
}

template class HoughTransformLaneDetector<float>;
template class HoughTransformLaneDetector<double>;
} // namespace Xycar
