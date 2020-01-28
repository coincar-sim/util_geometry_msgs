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

using namespace util_geometry_msgs::computations;

static const double DOUBLE_TOLERANCE = 10.e-9;

TEST(UtilGeometryMsgsComputations, calcDeltaPose) {
    geometry_msgs::Pose p0, p1, deltaPose;

    p0.position.x = 20.;
    p0.position.y = 0.;
    p0.position.z = 5.;
    p0.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 2.);

    p1.position.x = 10.;
    p1.position.y = 100.;
    p1.position.z = 1000.;
    p1.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI);

    deltaPose = util_geometry_msgs::computations::calcDeltaPose(p0, p1);

    EXPECT_NEAR(100., deltaPose.position.x, DOUBLE_TOLERANCE);
    EXPECT_NEAR(10., deltaPose.position.y, DOUBLE_TOLERANCE);
    EXPECT_NEAR(995., deltaPose.position.z, DOUBLE_TOLERANCE);

    double deltaYaw = util_geometry_msgs::conversions::yawFromPose(deltaPose);
    EXPECT_DOUBLE_EQ(M_PI / 2, deltaYaw);
}

TEST(UtilGeometryMsgsComputations, interpolateBetweenPoses) {
    geometry_msgs::Pose p0, p1, interp;

    p0.position.x = 1.;
    p0.position.y = 1.;
    p0.position.z = 1.;
    p0.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 2.);

    p1.position.x = 10.;
    p1.position.y = 100.;
    p1.position.z = 1000.;
    p1.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI);

    Eigen::Isometry3d p0Eigen, p1Eigen, interpEigen;
    util_geometry_msgs::conversions::fromMsg(p0, p0Eigen);
    util_geometry_msgs::conversions::fromMsg(p1, p1Eigen);

    interp = interpolateBetweenPoses(p0, p1, 0.);
    util_geometry_msgs::conversions::fromMsg(interp, interpEigen);
    EXPECT_TRUE(interpEigen.isApprox(p0Eigen));

    interp = interpolateBetweenPoses(p0, p1, 1.);
    util_geometry_msgs::conversions::fromMsg(interp, interpEigen);
    EXPECT_TRUE(interpEigen.isApprox(p1Eigen));

    interp = interpolateBetweenPoses(p0, p1, 0.1);
    EXPECT_DOUBLE_EQ(1.9, interp.position.x);
    EXPECT_DOUBLE_EQ(10.9, interp.position.y);
    EXPECT_DOUBLE_EQ(100.9, interp.position.z);

    double yaw_interp = util_geometry_msgs::conversions::yawFromPose(interp);
    EXPECT_DOUBLE_EQ(1.1 * (M_PI / 2.), yaw_interp);
}

TEST(UtilGeometryMsgsComputations, addDeltaPose) {
    geometry_msgs::Pose p0, p1, deltaPose;

    p0.position.x = 1.;
    p0.position.y = 0.;
    p0.position.z = 0.;
    p0.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 2.);

    deltaPose.position.x = 1.;
    deltaPose.position.y = 10.;
    deltaPose.position.z = 1000.;
    deltaPose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);

    p1 = addDeltaPose(p0, deltaPose);

    EXPECT_DOUBLE_EQ(-9., p1.position.x);
    EXPECT_NEAR(1., p1.position.y, DOUBLE_TOLERANCE);
    EXPECT_DOUBLE_EQ(1000., p1.position.z);

    double p1yaw = util_geometry_msgs::conversions::yawFromPose(p1);
    EXPECT_DOUBLE_EQ(0.75 * M_PI, p1yaw);
}

TEST(UtilGeometryMsgsComputations, subtractDeltaPose) {
    geometry_msgs::Pose p0, p1, deltaPose;

    p1.position.x = -9.;
    p1.position.y = 1.;
    p1.position.z = 1000.;
    p1.orientation = util_geometry_msgs::conversions::quaternionFromYaw(0.75 * M_PI);

    deltaPose.position.x = 1.;
    deltaPose.position.y = 10.;
    deltaPose.position.z = 1000.;
    deltaPose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(M_PI / 4.);

    p0 = subtractDeltaPose(p1, deltaPose);

    EXPECT_DOUBLE_EQ(1., p0.position.x);
    EXPECT_NEAR(0., p0.position.y, DOUBLE_TOLERANCE);
    EXPECT_DOUBLE_EQ(0., p0.position.z);

    double p0yaw = util_geometry_msgs::conversions::yawFromPose(p0);
    EXPECT_DOUBLE_EQ(0.5 * M_PI, p0yaw);
}
