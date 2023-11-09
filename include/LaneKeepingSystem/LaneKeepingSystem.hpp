// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.hpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class header file
 * @version 1.1
 * @date 2023-05-02
 */
#ifndef LANE_KEEPING_SYSTEM_HPP_
#define LANE_KEEPING_SYSTEM_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <ros/ros.h>
#include <string>
#include <tuple>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <xycar_msgs/xycar_motor.h>
#include <yaml-cpp/yaml.h>
#include <yolov3_trt_ros/BoundingBoxes.h>

#include "LaneKeepingSystem/BinaryFilter.hpp"
#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"
#include "LaneKeepingSystem/LaneKalmanFilter.hpp"
#include "LaneKeepingSystem/MovingAverageFilter.hpp"
#include "LaneKeepingSystem/PIDController.hpp"
#include "LaneKeepingSystem/StanleyController.hpp"
#include "LaneKeepingSystem/VehicleModel.hpp"

using DetectBoxs = std::priority_queue<std::pair<int32_t, std::pair<int16_t, ros::Time>>>;

namespace Xycar {
/**
 * @brief Lane Keeping System for searching and keeping Hough lines using Hough, Moving average and PID control
 *
 * @tparam Precision of data
 */
template <typename PREC>
class LaneKeepingSystem
{
public:
    using Ptr = std::unique_ptr<LaneKeepingSystem>;                     ///< Pointer type of this class
    using ControllerPtr = typename PIDController<PREC>::Ptr;            ///< Pointer type of PIDController
    using FilterPtr = typename MovingAverageFilter<PREC>::Ptr;          ///< Pointer type of MovingAverageFilter
    using DetectorPtr = typename HoughTransformLaneDetector<PREC>::Ptr; ///< Pointer type of LaneDetector
    using VehiclePtr = typename VehicleModel<PREC>::Ptr;
    using KalmanFilterPtr = typename LaneKalmanFilter<PREC>::Ptr;
    using StanleyPtr = typename StanleyController<PREC>::Ptr;
    using BinaryFilterPtr = typename BinaryFilter<PREC>::Ptr;

    static constexpr int32_t kXycarSteeringAangleLimit = 50; ///< Xycar Steering Angle Limit
    static constexpr double kFrameRate = 33.0;               ///< Frame rate
    /**
     * @brief Construct a new Lane Keeping System object
     */
    LaneKeepingSystem();

    /**
     * @brief Run Lane Keeping System
     */
    void run();

private:
    /**
     * @brief Set the parameters from config file
     *
     * @param[in] config Configuration for searching and keeping Hough lines using Hough, Moving average and PID control
     */
    void setParams(const YAML::Node& config);

    /**
     * @brief Control the speed of xycar
     *
     * @param[in] steeringAngle Angle to steer xycar. If over max angle, deaccelerate, otherwise accelerate
     */
    void speedControl(PREC steeringAngle);

    /**
     * @brief publish the motor topic message
     *
     * @param[in] steeringAngle Angle to steer xycar actually
     */
    void drive(PREC steeringAngle);

    void stop(PREC steeringAngle);

    /**
     * @brief Callback function for image topic
     *
     * @param[in] message Image topic message
     */
    void imageCallback(const sensor_msgs::Image& message);

    void liDARCallback(const sensor_msgs::LaserScan::ConstPtr& message);

    void trafficSignCallback(const yolov3_trt_ros::BoundingBoxes& message);

private:
    ControllerPtr mPID;                      ///< PID Class for Control
    FilterPtr mMovingAverage;                ///< Moving Average Filter Class for Noise filtering
    DetectorPtr mHoughTransformLaneDetector; ///< Hough Transform Lane Detector Class for Lane Detection
    VehiclePtr mVehicleModel;
    KalmanFilterPtr mKalmanFilter;
    BinaryFilterPtr mBinaryFilter;
    StanleyPtr mStanley;

    // ROS Variables
    ros::NodeHandle mNodeHandler; ///< Node Hanlder for ROS. In this case Detector and Controler
    ros::Publisher mPublisher;    ///< Publisher to send message about
    ros::Subscriber mSubscriber;  ///< Subscriber to receive image
    ros::Subscriber mLidarSubscriber;
    ros::Subscriber mTrafficSignSubscriber;

    ros::Publisher mVehicleStatePublisher;
    ros::Publisher mLanePositionPublisher;

    std::string mPublishingTopicName;      ///< Topic name to publish
    std::string mSubscribedTopicName;      ///< Topic name to subscribe
    uint32_t mQueueSize;                   ///< Max queue size for message
    xycar_msgs::xycar_motor mMotorMessage; ///< Message for the motor of xycar

    // OpenCV Image processing Variables
    cv::Mat mFrame; ///< Image from camera. The raw image is converted into cv::Mat
    std::vector<std::string> mDetectionLabel;

    DetectBoxs mDetectTrafficSigns;
    int16_t mTrafficSignLabel;

    // Xycar Device variables
    PREC mXycarSpeed;                 ///< Current speed of xycar
    PREC mXycarMaxSpeed;              ///< Max speed of xycar
    PREC mXycarMinSpeed;              ///< Min speed of xycar
    PREC mXycarSpeedControlThreshold; ///< Threshold of angular of xycar
    PREC mAccelerationStep;           ///< How much would accelrate xycar depending on threshold
    PREC mDecelerationStep;           ///< How much would deaccelrate xycar depending on threshold

    PREC mStanleyGain;
    PREC mStanleyLookAheadDistance;

    PREC mStopSampleSize;
    PREC mStopProbability;

    PREC mLinearUnit;
    PREC mAngleUnit;

    PREC mRotateThreshold;

    PREC mLidarAngleThreshold;
    PREC mLidarDistanceThreshold;
    PREC mLidarClusterThreshold;
    PREC mLidarClusterMinpoint;
    PREC mLidarClusterMaxpoint;
    PREC mLidarTrackingThreshold;
    PREC mLidarTrackingMissSecond;

    PREC mSignSignalSecond;

    PREC mMinBoundingboxArea;

    std::pair<PREC, PREC> mAvoidanceInput;
    std::pair<PREC, PREC> mRotateInput;
    std::pair<PREC, PREC> mSignInput;

    std::vector<std::tuple<PREC, PREC, PREC, PREC, ros::Time>> mLidarDetectBox;

    // Debug Flag
    bool mDebugging; ///< Debugging or not
};
} // namespace Xycar

#endif // LANE_KEEPING_SYSTEM_HPP_
