/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "gtest/gtest.h"

#define EXPECT_COVARIANCE_NEAR(expected, actual, abs_err)                     \
    {                                                                         \
        ASSERT_EQ(36u, expected.size()) << "A covariance must have size 36!"; \
        ASSERT_EQ(36u, actual.size()) << "A covariance must have size 36!";   \
        for (std::size_t i = 0; i < 36u; ++i) {                               \
            EXPECT_NEAR(expected[i], actual[i], abs_err);                     \
        }                                                                     \
    }

#define EXPECT_POSE_NEAR(expected, actual, abs_error)                         \
    {                                                                         \
        EXPECT_NEAR(expected.position.x, actual.position.x, abs_error);       \
        EXPECT_NEAR(expected.position.y, actual.position.y, abs_error);       \
        EXPECT_NEAR(expected.position.z, actual.position.z, abs_error);       \
        EXPECT_NEAR(expected.orientation.x, actual.orientation.x, abs_error); \
        EXPECT_NEAR(expected.orientation.y, actual.orientation.y, abs_error); \
        EXPECT_NEAR(expected.orientation.z, actual.orientation.z, abs_error); \
        EXPECT_NEAR(expected.orientation.w, actual.orientation.w, abs_error); \
    }

#define EXPECT_POSE_WITH_COVARIANCE_NEAR(expected, actual, abs_error)              \
    {                                                                              \
        EXPECT_POSE_NEAR(expected.pose, actual.pose, abs_error);                   \
        EXPECT_COVARIANCE_NEAR(expected.covariance, actual.covariance, abs_error); \
    }

#define EXPECT_TWIST_NEAR(expected, actual, abs_error)                \
    {                                                                 \
        EXPECT_NEAR(expected.linear.x, actual.linear.x, abs_error);   \
        EXPECT_NEAR(expected.linear.y, actual.linear.y, abs_error);   \
        EXPECT_NEAR(expected.linear.z, actual.linear.z, abs_error);   \
        EXPECT_NEAR(expected.angular.x, actual.angular.x, abs_error); \
        EXPECT_NEAR(expected.angular.y, actual.angular.y, abs_error); \
        EXPECT_NEAR(expected.angular.z, actual.angular.z, abs_error); \
    }

#define EXPECT_TWIST_WITH_COVARIANCE_NEAR(expected, actual, abs_error)             \
    {                                                                              \
        EXPECT_TWIST_NEAR(expected.twist, actual.twist, abs_error);                \
        EXPECT_COVARIANCE_NEAR(expected.covariance, actual.covariance, abs_error); \
    }


#include "util_geometry_msgs.hpp"

using namespace util_geometry_msgs::transformations;
using CovarianceType = geometry_msgs::PoseWithCovariance::_covariance_type;

static const double DoubleTolerance = 10.e-9;

class UtilGeometryMsgsTransformations : public ::testing::Test {
protected:
    // NOLINTNEXTLINE(readability-function-size)
    void SetUp() override {
        std::srand(42); // used fixed seed

        covGroundTruthValues = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        covArbitraryDiagValuesValid = {1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0,
                                       0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 6};

        covArbitraryValuesValid = {11, 12, 13, 14, 15, 16, 12, 22, 23, 24, 25, 26, 13, 23, 33, 34, 35, 36,
                                   14, 24, 34, 44, 45, 46, 15, 25, 35, 45, 55, 56, 16, 26, 36, 46, 56, 66};

        p00.pose.position.x = 0.0;
        p00.pose.position.y = 0.0;
        p00.pose.position.z = 0.0;
        p00.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(0.);
        p00.covariance = covArbitraryDiagValuesValid;
        poses.push_back(p00);

        p130.pose.position.x = 1.0;
        p130.pose.position.y = 1.0;
        p130.pose.position.z = 1.0;
        p130.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 3.);
        p130.covariance = covArbitraryDiagValuesValid;
        poses.push_back(p130);

        p10.pose.position.x = 1.0;
        p10.pose.position.y = 1.0;
        p10.pose.position.z = 1.0;
        p10.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(0.);
        p10.covariance = covArbitraryDiagValuesValid;
        poses.push_back(p10);

        t00.twist.linear.x = 0.0;
        t00.twist.linear.y = 0.0;
        t00.twist.linear.z = 0.0;
        t00.twist.angular.x = 0.0;
        t00.twist.angular.y = 0.0;
        t00.twist.angular.z = 0.0;
        t00.covariance = covArbitraryDiagValuesValid;
        twists.push_back(t00);

        t10.twist.linear.x = 1.0;
        t10.twist.linear.y = 1.0;
        t10.twist.linear.z = 1.0;
        t10.twist.angular.x = 0.0;
        t10.twist.angular.y = 0.0;
        t10.twist.angular.z = 0.0;
        t10.covariance = covArbitraryDiagValuesValid;
        twists.push_back(t10);

        t123.twist.linear.x = 1.0;
        t123.twist.linear.y = 2.0;
        t123.twist.linear.z = 3.0;
        t123.twist.angular.x = 1.0;
        t123.twist.angular.y = 2.0;
        t123.twist.angular.z = 3.0;
        t123.covariance = covArbitraryDiagValuesValid;
        twists.push_back(t123);

        a00.accel.linear.x = 0.0;
        a00.accel.linear.y = 0.0;
        a00.accel.linear.z = 0.0;
        a00.accel.angular.x = 0.0;
        a00.accel.angular.y = 0.0;
        a00.accel.angular.z = 0.0;
        a00.covariance = covArbitraryDiagValuesValid;
        accels.push_back(a00);

        a10.accel.linear.x = 1.0;
        a10.accel.linear.y = 1.0;
        a10.accel.linear.z = 1.0;
        a10.accel.angular.x = 0.0;
        a10.accel.angular.y = 0.0;
        a10.accel.angular.z = 0.0;
        a10.covariance = covArbitraryDiagValuesValid;
        accels.push_back(a10);

        a123.accel.linear.x = 1.0;
        a123.accel.linear.y = 2.0;
        a123.accel.linear.z = 3.0;
        a123.accel.angular.x = 1.0;
        a123.accel.angular.y = 2.0;
        a123.accel.angular.z = 3.0;
        a123.covariance = covArbitraryDiagValuesValid;
        accels.push_back(a123);

        // define base_frame
        baseFrame.position.x = 0.0;
        baseFrame.position.y = 0.0;
        baseFrame.position.z = 0.0;
        baseFrame.orientation = util_geometry_msgs::conversions::quaternionFromYaw(0.);
    }

    CovarianceType covGroundTruthValues{};
    CovarianceType covArbitraryDiagValuesValid{};
    CovarianceType covArbitraryValuesValid{};

    geometry_msgs::PoseWithCovariance p00{};
    geometry_msgs::PoseWithCovariance p10{};
    geometry_msgs::PoseWithCovariance p130{};
    geometry_msgs::TwistWithCovariance t00{};
    geometry_msgs::TwistWithCovariance t10{};
    geometry_msgs::TwistWithCovariance t123{};

    geometry_msgs::AccelWithCovariance a00{};
    geometry_msgs::AccelWithCovariance a10{};
    geometry_msgs::AccelWithCovariance a123{};

    std::vector<geometry_msgs::PoseWithCovariance> poses{};
    std::vector<geometry_msgs::TwistWithCovariance> twists{};
    std::vector<geometry_msgs::AccelWithCovariance> accels{};

    geometry_msgs::Pose baseFrame{};
};

// NOLINTNEXTLINE(readability-function-size)
TEST_F(UtilGeometryMsgsTransformations, SameFrameReferencingTestPose) {
    for (geometry_msgs::PoseWithCovariance& frame : poses) {
        for (geometry_msgs::PoseWithCovariance& p : poses) {
            geometry_msgs::PoseWithCovariance& origPose = p;
            geometry_msgs::PoseWithCovariance rereferencedPose;
            rereferencePoseWithCovariance(origPose, frame.pose, frame.pose, rereferencedPose);
            EXPECT_POSE_WITH_COVARIANCE_NEAR(origPose, rereferencedPose, DoubleTolerance);
        }
    }
}

// NOLINTNEXTLINE(readability-function-size)
TEST_F(UtilGeometryMsgsTransformations, SameFrameReferencingTestTwist) {
    for (geometry_msgs::PoseWithCovariance& frame : poses) {
        for (geometry_msgs::TwistWithCovariance& t : twists) {
            geometry_msgs::TwistWithCovariance& origTwist = t;
            geometry_msgs::TwistWithCovariance rereferencedTwist;
            rereferenceTwistWithCovariance(origTwist, frame.pose, frame.pose, rereferencedTwist);
            EXPECT_TWIST_WITH_COVARIANCE_NEAR(origTwist, rereferencedTwist, DoubleTolerance);
        }
    }
}

TEST_F(UtilGeometryMsgsTransformations, SelfReferencingTestPose) {
    for (geometry_msgs::PoseWithCovariance& p : poses) {
        geometry_msgs::PoseWithCovariance& origPose = p;
        geometry_msgs::PoseWithCovariance& frame = p;
        geometry_msgs::PoseWithCovariance rereferencedPose;
        rereferencePoseWithCovariance(origPose, baseFrame, frame.pose, rereferencedPose);
        EXPECT_POSE_NEAR(p00.pose, rereferencedPose.pose, DoubleTolerance);
    }
}

// NOLINTNEXTLINE(readability-function-size)
TEST_F(UtilGeometryMsgsTransformations, ninetyDegreeReferencingTestPose) {

    geometry_msgs::PoseWithCovariance pOrig, pExpected, pActual;

    geometry_msgs::Pose frame;
    frame.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 2.);

    // first -90 degree around z
    pOrig.pose.position.x = 1;
    pOrig.pose.position.y = 0;
    pOrig.pose.position.z = 0;
    pOrig.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(0.);
    pOrig.covariance = covGroundTruthValues;

    pExpected.pose.position.x = 0;
    pExpected.pose.position.y = -1;
    pExpected.pose.position.z = 0;
    pExpected.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(-M_PI / 2.);
    pExpected.covariance = covGroundTruthValues;

    rereferencePoseWithCovariance(pOrig, baseFrame, frame, pActual);
    EXPECT_POSE_WITH_COVARIANCE_NEAR(pExpected, pActual, DoubleTolerance);
}


TEST(UtilGeometryMsgsTransformationsTest, DifferentFramesRereferencingTestTwist) {
    geometry_msgs::Pose poseTo;
    poseTo.position.x = 353.0;
    poseTo.position.y = 13231.0;
    poseTo.position.z = 92897.0;
    poseTo.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 2.);

    geometry_msgs::Pose poseFrom;
    poseFrom.position.x = 0.0;
    poseFrom.position.y = 0.0;
    poseFrom.position.z = 0.0;
    poseFrom.orientation = util_geometry_msgs::conversions::quaternionFromYaw(0.);

    geometry_msgs::Twist origTwist;
    origTwist.linear.x = 1.0;
    origTwist.linear.y = 0.0;
    origTwist.linear.z = 0.0;
    origTwist.angular.x = 0.0;
    origTwist.angular.y = 0.5;
    origTwist.angular.z = 0.0;

    geometry_msgs::Twist rereferencedTwistExpected;
    rereferencedTwistExpected.linear.x = 0.0;
    rereferencedTwistExpected.linear.y = -1.0;
    rereferencedTwistExpected.linear.z = 0.0;
    rereferencedTwistExpected.angular.x = 0.5;
    rereferencedTwistExpected.angular.y = 0.0;
    rereferencedTwistExpected.angular.z = 0.0;

    geometry_msgs::Twist rereferencedTwist;
    rereferenceTwist(origTwist, poseFrom, poseTo, rereferencedTwist);
    EXPECT_TWIST_NEAR(rereferencedTwistExpected, rereferencedTwist, DoubleTolerance);
}
