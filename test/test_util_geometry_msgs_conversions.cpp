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

#include "util_geometry_msgs.hpp"

using namespace util_geometry_msgs::conversions;

static const double DOUBLE_TOLERANCE = 10.e-9;

TEST(UtilGeometryMsgsConversions, quaternionFromYaw) {
    geometry_msgs::Quaternion quat;
    quat = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 2.);
    EXPECT_NEAR(0., quat.x, DOUBLE_TOLERANCE);
    EXPECT_NEAR(0., quat.y, DOUBLE_TOLERANCE);
    EXPECT_NEAR(1. / sqrt(2.), quat.z, DOUBLE_TOLERANCE);
    EXPECT_NEAR(1. / sqrt(2.), quat.w, DOUBLE_TOLERANCE);

    quat = util_geometry_msgs::conversions::quaternionFromYaw(-M_PI / 4.);
    EXPECT_NEAR(0., quat.x, DOUBLE_TOLERANCE);
    EXPECT_NEAR(0., quat.y, DOUBLE_TOLERANCE);
    EXPECT_NEAR(-0.3826834323, quat.z, DOUBLE_TOLERANCE);
    EXPECT_NEAR(0.9238795325, quat.w, DOUBLE_TOLERANCE);
}

TEST(UtilGeometryMsgsConversions, pose2Isometry) {
    geometry_msgs::Pose pose;
    pose.position.x = 1.;
    pose.position.y = 10.;
    pose.position.z = 100.;

    pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);

    Eigen::Isometry3d eigenPose;
    util_geometry_msgs::conversions::fromMsg(pose, eigenPose);
    EXPECT_NEAR(1. / sqrt(2.), eigenPose(0, 0), DOUBLE_TOLERANCE);
    EXPECT_NEAR(1. / sqrt(2.), eigenPose(1, 0), DOUBLE_TOLERANCE);
    EXPECT_NEAR(-1. / sqrt(2.), eigenPose(0, 1), DOUBLE_TOLERANCE);
    EXPECT_NEAR(1. / sqrt(2.), eigenPose(1, 1), DOUBLE_TOLERANCE);

    EXPECT_NEAR(1, eigenPose(0, 3), DOUBLE_TOLERANCE);
    EXPECT_NEAR(10, eigenPose(1, 3), DOUBLE_TOLERANCE);
    EXPECT_NEAR(100, eigenPose(2, 3), DOUBLE_TOLERANCE);
}

TEST(UtilGeometryMsgsConversions, isometry2Pose) {
    Eigen::Isometry3d eigenPose;

    eigenPose.setIdentity();
    eigenPose.translate(Eigen::Vector3d(1., 2., 3.));
    eigenPose.rotate(Eigen::AngleAxisd(M_PI / 4., Eigen::Vector3d::UnitZ()));

    geometry_msgs::Pose pose = util_geometry_msgs::conversions::toMsg(eigenPose);

    EXPECT_DOUBLE_EQ(1., pose.position.x);
    EXPECT_DOUBLE_EQ(2., pose.position.y);
    EXPECT_DOUBLE_EQ(3., pose.position.z);
    EXPECT_NEAR(M_PI / 4., yawFromQuaternion(pose.orientation), DOUBLE_TOLERANCE);
}

TEST(UtilGeometryMsgsConversions, quat2EigenQuat) {
    geometry_msgs::Quaternion quat;
    quat = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);

    Eigen::Quaterniond eigenQuat;
    fromMsg(quat, eigenQuat);
    EXPECT_NEAR(quat.x, eigenQuat.x(), DOUBLE_TOLERANCE);
    EXPECT_NEAR(quat.y, eigenQuat.y(), DOUBLE_TOLERANCE);
    EXPECT_NEAR(quat.z, eigenQuat.z(), DOUBLE_TOLERANCE);
    EXPECT_NEAR(quat.w, eigenQuat.w(), DOUBLE_TOLERANCE);
}

TEST(UtilGeometryMsgsConversions, yawFromQuaternion) {
    geometry_msgs::Quaternion quat;
    quat = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);

    double yaw = yawFromQuaternion(quat);
    EXPECT_NEAR(M_PI / 4., yaw, DOUBLE_TOLERANCE);
}

TEST(UtilGeometryMsgsConversions, boostArrayToEigenMatrix) {
    boost::array<double, 4> array{1., 2., 3., 4.};
    Eigen::Matrix2d matrix;

    boostArrayToEigenMatrix(array, matrix, true);
    EXPECT_DOUBLE_EQ(1., matrix(0, 0));
    EXPECT_DOUBLE_EQ(2., matrix(0, 1));

    boostArrayToEigenMatrix(array, matrix, false);
    EXPECT_DOUBLE_EQ(1., matrix(0, 0));
    EXPECT_DOUBLE_EQ(2., matrix(1, 0));
}

TEST(UtilGeometryMsgsConversions, eigenMatrixToBoostArray) {
    Eigen::Matrix2d matrix;
    matrix << 1, 2, 3, 4;

    boost::array<double, 4> array;

    eigenMatrixToBoostArray(matrix, array, true);
    EXPECT_DOUBLE_EQ(1., array.at(0));
    EXPECT_DOUBLE_EQ(2., array.at(1));

    eigenMatrixToBoostArray(matrix, array, false);
    EXPECT_DOUBLE_EQ(1., array.at(0));
    EXPECT_DOUBLE_EQ(2., array.at(2));
}
