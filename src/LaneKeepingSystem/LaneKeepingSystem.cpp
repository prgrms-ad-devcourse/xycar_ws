// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class source file
 * @version 1.1
 * @date 2023-05-02
 */
#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = std::make_unique<PIDController<PREC>>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = std::make_unique<MovingAverageFilter<PREC>>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mHoughTransformLaneDetector = std::make_unique<HoughTransformLaneDetector<PREC>>(config);
    mVehicleModel = std::make_unique<VehicleModel<PREC>>(0, 0, 0);
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
    mLidarSubscriber = mNodeHandler.subscribe("/scan", 1, &LaneKeepingSystem::liDARCallback, this);
    mTrafficSignSubscriber = mNodeHandler.subscribe("/yolov3_trt_ros/detections", 1, &LaneKeepingSystem::trafficSignCallback, this);

    mVehicleStatePublisher = mNodeHandler.advertise<std_msgs::Float32MultiArray>("/vehicle_state", 1);
    mLanePositionPublisher = mNodeHandler.advertise<std_msgs::Float32MultiArray>("/lane_position", 1);

    mStanley = std::make_unique<StanleyController<PREC>>(mStanleyGain, mStanleyLookAheadDistance);
    mBinaryFilter = std::make_unique<BinaryFilter<PREC>>(mStopSampleSize, mStopProbability);
    mTrafficSignLabel = -1;
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    mStanleyGain = config["STANLEY"]["K_GAIN"].as<PREC>();
    mStanleyLookAheadDistance = config["STANLEY"]["LOOK_AHREAD_DISTANCE"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();

    mLinearUnit = config["XYCAR"]["LINEAR_UNIT"].as<PREC>();
    mAngleUnit = config["XYCAR"]["ANGLE_UNIT"].as<PREC>();
    mStopSampleSize = config["STOP"]["SAMPLE_SIZE"].as<int32_t>();
    mStopProbability = config["STOP"]["PROBABILITY"].as<PREC>();

    mDetectionLabel = config["DETECTION"]["CLASSES"].as<std::vector<std::string>>();
    mLidarAngleThreshold = config["LIDAR"]["RANGE_RADIAN_THRESHOLD"].as<PREC>();
    mLidarDistanceThreshold = config["LIDAR"]["RANGE_DISTANCE_THRESHOLD"].as<PREC>();
    mLidarClusterThreshold = config["LIDAR"]["CLUSTER_DISTANCE_THRESHOLD"].as<PREC>();
    mLidarClusterMinpoint = config["LIDAR"]["CLUSTER_MIN_POINT"].as<PREC>();
    mLidarClusterMaxpoint = config["LIDAR"]["CLUSTER_MAX_POINT"].as<PREC>();
    mLidarTrackingThreshold = config["LIDAR"]["TRACKING_DISTANCE_THRESHOLD"].as<PREC>();
    mLidarTrackingMissSecond = config["LIDAR"]["TRACKING_MISS_SECOND"].as<PREC>();

    mSignSignalSecond = config["DETECTION"]["SIGN_LIFESECOND"].as<PREC>();

    mAvoidanceInput = std::make_pair(config["AVOIDANCE_INPUT"]["POSITION"].as<PREC>(), config["AVOIDANCE_INPUT"]["SLOPE"].as<PREC>());
    mRotateInput = std::make_pair(config["ROTATE_INPUT"]["POSITION"].as<PREC>(), config["ROTATE_INPUT"]["SLOPE"].as<PREC>());
    mSignInput = std::make_pair(config["SIGN_INPUT"]["POSITION"].as<PREC>(), config["SIGN_INPUT"]["SLOPE"].as<PREC>());

    mRotateThreshold = config["XYCAR"]["ROTATE_THRESHOLD"].as<PREC>();
    mMinBoundingboxArea = config["DETECTION"]["MIN_BOUNDINGBOX_AREA"].as<PREC>();
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    const PREC PI = std::atan(1) * 4.0;
    ros::Rate rate(kFrameRate);
    ros::Time currentTime, previousTime, pubTime, stopTime;
    ros::Time now = ros::Time::now();
    PREC previousSteeringAngle = 0.f;
    PREC steeringMaxRadian = kXycarSteeringAangleLimit * (PI / 180.f);

    std::cout << "max radian: " << steeringMaxRadian << std::endl;
    bool enableStopDetected = true;
    bool previousStopDetected = false;
    bool stopDetected = false;
    bool setStopTimer = true;
    bool isStop = false;
    bool runUpdate = true;

    std::string detectedTrafficSignLabel = "IGNORE";
    std::string prevDetectedTrafficSignLabel = "IGNORE";
    Eigen::Vector2d inputVector;
    Eigen::Vector2d zeroVector;

    currentTime = now;
    previousTime = now;
    pubTime = now;
    stopTime = now;

    inputVector << 0.f, 0.f;

    while (ros::ok())
    {
        ros::spinOnce();

        if (mFrame.empty())
            continue;

        mHoughTransformLaneDetector->copyDebugFrame(mFrame);

        if (!mDetectTrafficSigns.empty())
        {
            const auto [boundingBoxArea, box] = mDetectTrafficSigns.top();
            const auto [trafficSignLabel, trafficSignTime] = box;

            detectedTrafficSignLabel = mDetectionLabel[trafficSignLabel];
        } else {
            detectedTrafficSignLabel = "IGNORE";
        }

        int32_t leftPositionX = 0;
        int32_t rightPositionX = 640;

        PREC steeringAngle = 0.f;

        if (mLidarDetectBox.size() > 0)
        {
            std::cout << "detect box length: " << mLidarDetectBox.size() << std::endl;
            const auto [positionInput, slopeInput] = mAvoidanceInput;
            auto bbox = mLidarDetectBox[0];
            PREC shortPointRadian = std::atan2(mLidarDistanceThreshold - (std::get<2>(bbox) + std::get<0>(bbox)) / 2.f, (std::get<3>(bbox) + std::get<1>(bbox)) / 2.f);
            PREC PI_HALF = PI / 2.f;

            // RIGHT LEFT
            if (shortPointRadian < PI / 2.f)
            {
                steeringAngle = steeringMaxRadian * (shortPointRadian / PI_HALF);
                inputVector << positionInput * -1, slopeInput * -1;
            }
            else
            {
                steeringAngle = steeringMaxRadian * ((PI_HALF - (shortPointRadian - PI_HALF)) / PI_HALF) * -1;
                inputVector << positionInput, slopeInput;
            }
            runUpdate = false;
            steeringAngle = static_cast<PREC>(steeringAngle * (180 / PI));
            std::cout << "Lidar Driving Angle: " << steeringAngle << std::endl;

            auto [predictLeftPositionX, predictRightPositionX] = mHoughTransformLaneDetector->predictLanePosition(inputVector);

            leftPositionX = predictLeftPositionX;
            rightPositionX = predictRightPositionX;
        }
        else
        {
            runUpdate = true;
            inputVector << 0.f, 0.f;
            const auto [positionInput, slopeInput] = mRotateInput;

            if (previousSteeringAngle < mRotateThreshold * -1)
            {
                inputVector << positionInput * -1, slopeInput * -1;
            }
            else if (previousSteeringAngle > mRotateThreshold)
            {
                inputVector << positionInput, slopeInput;
            }
            else if (detectedTrafficSignLabel == "LEFT")
            {
                inputVector << mSignInput.first * -1, mSignInput.second * -1;
            }
            else if (detectedTrafficSignLabel == "RIGHT")
            {
                inputVector << mSignInput.first, mSignInput.second;
            }

            std::cout << "PREV DETECTED: " << prevDetectedTrafficSignLabel << std::endl;
            std::cout << "DETECTED: " << detectedTrafficSignLabel << std::endl;
            std::cout << "RUN UPDATE: " << runUpdate << std::endl;
            // std::cout << "input vector: " << inputVector(0) << std::endl;
            mHoughTransformLaneDetector->predictLanePosition(inputVector);
            auto [predictLeftPositionX, predictRightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame, runUpdate, detectedTrafficSignLabel);

            leftPositionX = predictLeftPositionX;
            rightPositionX = predictRightPositionX;

            currentTime = ros::Time::now();
            stopDetected = mHoughTransformLaneDetector->getStopLineStatus();

            mBinaryFilter->addSample(stopDetected);
            PREC stopProbability = mBinaryFilter->getResult();
            stopDetected = stopProbability > 0.5;

            ros::Duration delta_t = currentTime - previousTime;
            mVehicleModel->update(mXycarSpeed / mLinearUnit, previousSteeringAngle * (M_PI / 180.f), static_cast<double>(delta_t.toNSec()) / 1000000.f / 1000.f);

            int32_t estimatedPositionX = static_cast<int32_t>((leftPositionX + rightPositionX) / 2);
            int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
            mStanley->calculateSteeringAngle(errorFromMid, 0, mXycarSpeed);

            PREC stanleyResult = mStanley->getResult();
            steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(stanleyResult), static_cast<PREC>(kXycarSteeringAangleLimit)));

            // std::cout << "error: " << errorFromMid << std::endl;
        }

        std::cout << "position: " << leftPositionX << ", " << rightPositionX << std::endl;
        // std::cout << "steeringAngle: " << steeringAngle << std::endl;

        // std::cout << "sign: " << detectedTrafficSignLabel << std::endl;
        // std::cout << "input vector: " << inputVector(0) << std::endl;
        // std::cout << "stop: " << stopProbability << std::endl;

        if (enableStopDetected && stopDetected && !previousStopDetected)
        {
            stopTime = currentTime;
            setStopTimer = false;
            isStop = true;
            enableStopDetected = false;
        }

        int32_t estimatedPositionX = static_cast<int32_t>((leftPositionX + rightPositionX) / 2);

        speedControl(steeringAngle);
        drive(steeringAngle);

        previousSteeringAngle = steeringAngle;
        previousStopDetected = stopDetected;
        prevDetectedTrafficSignLabel = detectedTrafficSignLabel;

        if (mDebugging)
        {
            std_msgs::Float32MultiArray vehicleStateMsg;

            std::tuple<PREC, PREC, PREC> vehicleState = mVehicleModel->getResult();

            vehicleStateMsg.data.push_back(std::get<0>(vehicleState));
            vehicleStateMsg.data.push_back(std::get<1>(vehicleState));
            vehicleStateMsg.data.push_back(std::get<2>(vehicleState));
            vehicleStateMsg.data.push_back(leftPositionX);
            vehicleStateMsg.data.push_back(rightPositionX);

            ros::Duration pubDiff = ros::Time::now() - pubTime;

            if (pubDiff.toSec() > 0.1)
            {
                mVehicleStatePublisher.publish(vehicleStateMsg);
                pubTime = ros::Time::now();
            }

            mHoughTransformLaneDetector->drawRectangles(leftPositionX, rightPositionX, estimatedPositionX);
            cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());
            cv::waitKey(1);
        }

        previousTime = ros::Time::now();
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::liDARCallback(const sensor_msgs::LaserScan::ConstPtr& message)
{
    const PREC angleIncrement = message->angle_increment;
    const PREC radianThreshold = mLidarAngleThreshold;
    const PREC distanceThreshold = mLidarDistanceThreshold;

    const PREC clusterDistanceThreshold = mLidarClusterThreshold;
    const PREC radian2 = 6.28319;

    std::vector<std::array<PREC, 2>> XYLidarPoints;
    std::vector<std::tuple<PREC, PREC, PREC, PREC, ros::Time>> lidarDetectBoxs;
    std::vector<std::tuple<PREC, PREC, PREC, PREC, ros::Time>> lidarUpdateBoxs;

    for (int32_t i = 0; i < message->ranges.size(); i++)
    {
        PREC distance = message->ranges[i];
        PREC angle = static_cast<PREC>(i) * angleIncrement;
        PREC x = distance * std::cos(angle);
        PREC y = distance * std::sin(angle);
        std::array<PREC, 2> pointXY = { x, y };

        if (distance > 0 && distance < distanceThreshold && (angle < radianThreshold || angle > radian2 - radianThreshold))
        {
            XYLidarPoints.push_back(pointXY);
        }
    }

    const int32_t pointSize = XYLidarPoints.size();
    std::vector<bool> visited(pointSize, false);

    for (uint32_t i = 0; i < pointSize; i++)
    {
        std::vector<int> cluster;
        std::queue<int> queue;

        if (visited[i])
        {
            continue;
        }

        visited[i] = true;
        queue.push(i);

        while (!queue.empty())
        {
            int currentIndex = queue.front();
            auto pointXY = XYLidarPoints[currentIndex];
            queue.pop();
            cluster.push_back(currentIndex);

            for (uint32_t j = 0; j < pointSize; j++)
            {
                if (visited[j])
                {
                    continue;
                }
                auto targetPointXY = XYLidarPoints[j];
                PREC dx = pointXY[0] - targetPointXY[0];
                PREC dy = pointXY[1] - targetPointXY[1];
                PREC distance = std::sqrt(dx * dx + dy * dy);

                if (distance <= clusterDistanceThreshold)
                {
                    queue.push(j);
                    visited[j] = true;
                }
            }
        }
        if (cluster.size() > mLidarClusterMinpoint && cluster.size() < mLidarClusterMaxpoint)
        {
            std::array<PREC, 4> detectedBox = { 10000.f, 10000.f, -1.f, -1.f };
            for (auto index : cluster)
            {
                detectedBox[0] = std::min(detectedBox[0], XYLidarPoints[index][0]);
                detectedBox[1] = std::min(detectedBox[1], XYLidarPoints[index][1]);
                detectedBox[2] = std::max(detectedBox[2], XYLidarPoints[index][0]);
                detectedBox[3] = std::max(detectedBox[3], XYLidarPoints[index][1]);
            }

            lidarDetectBoxs.push_back(std::make_tuple(detectedBox[0], detectedBox[1], detectedBox[2], detectedBox[3], ros::Time::now()));
        }
    }

    ros::Time nowTime = ros::Time::now();

    if (mLidarDetectBox.size() <= 0)
    {
        mLidarDetectBox = lidarDetectBoxs;
    }
    else
    {
        for (int32_t i = 0; i < mLidarDetectBox.size(); i++)
        {
            auto pBox = mLidarDetectBox[i];
            ros::Duration timeDiff = nowTime - std::get<4>(pBox);

            PREC prevX = (std::get<2>(pBox) + std::get<0>(pBox)) / 2.f;
            PREC prevY = (std::get<3>(pBox) + std::get<1>(pBox)) / 2.f;
            bool isUpdate = false;
            for (auto box : lidarDetectBoxs)
            {
                PREC nowX = (std::get<2>(box) + std::get<0>(box)) / 2.f;
                PREC nowY = (std::get<3>(box) + std::get<1>(box)) / 2.f;
                PREC dx = nowX - prevX;
                PREC dy = nowY - prevY;
                PREC distance = std::sqrt(dx * dx + dy * dy);

                if (distance <= mLidarTrackingThreshold)
                {
                    isUpdate = true;
                    lidarUpdateBoxs.push_back(box);
                    break;
                }
            }

            if (!isUpdate && timeDiff.toSec() <= mLidarTrackingMissSecond)
            {
                lidarUpdateBoxs.push_back(pBox);
            }
        }
        mLidarDetectBox = lidarUpdateBoxs;
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::trafficSignCallback(const yolov3_trt_ros::BoundingBoxes& message)
{
    DetectBoxs DetectTrafficSigns;
    ros::Time now = ros::Time::now();
    std::vector<std::tuple<int32_t, int16_t, ros::Time>> trafficSigns;

    for (auto boundingBox : message.bounding_boxes)
    {
        int32_t minX = boundingBox.xmin;
        int32_t minY = boundingBox.ymin;
        int32_t maxX = boundingBox.xmax;
        int32_t maxY = boundingBox.ymax;
        int32_t boundingBoxArea = (maxX - minX) * (maxY - minY);

        if (boundingBoxArea <= mMinBoundingboxArea)
        {
            continue;
        }

        trafficSigns.push_back(std::make_tuple(boundingBoxArea, boundingBox.id, now));
    }

    if (mDetectTrafficSigns.empty())
    {
        for (auto trafficSign : trafficSigns)
        {
            DetectTrafficSigns.push(std::make_pair(std::get<0>(trafficSign), std::make_pair(std::get<1>(trafficSign), std::get<2>(trafficSign))));
        }
    }
    else
    {
        while (!mDetectTrafficSigns.empty())
        {
            bool isUpdate = false;
            const auto [boundingBoxArea, box] = mDetectTrafficSigns.top();
            const auto [trafficSignLabel, trafficSignTime] = box;

            ros::Duration timeDiff = now - trafficSignTime;
            mDetectTrafficSigns.pop();

            for (auto trafficSign : trafficSigns)
            {
                if (std::abs(boundingBoxArea - std::get<0>(trafficSign)) <= boundingBoxArea * 0.3 && trafficSignLabel == std::get<1>(trafficSign))
                {
                    DetectTrafficSigns.push(std::make_pair(std::get<0>(trafficSign), std::make_pair(std::get<1>(trafficSign), std::get<2>(trafficSign))));
                    isUpdate = true;
                    break;
                }
            }

            if (!isUpdate && timeDiff.toSec() <= mSignSignalSecond)
            {
                DetectTrafficSigns.push(std::make_pair(boundingBoxArea, std::make_pair(trafficSignLabel, trafficSignTime)));
            }
        }
    }
    mDetectTrafficSigns = DetectTrafficSigns;
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::stop(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;

    motorMessage.header.stamp = ros::Time::now();
    motorMessage.angle = steeringAngle;
    motorMessage.speed = 0;

    mPublisher.publish(motorMessage);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;

    motorMessage.header.stamp = ros::Time::now();
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);

    // motorMessage.angle = 0;
    // motorMessage.speed = 5;

    mPublisher.publish(motorMessage);
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
