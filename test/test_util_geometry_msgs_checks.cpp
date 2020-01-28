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

using namespace util_geometry_msgs::checks;
using covariance_type = geometry_msgs::PoseWithCovariance::_covariance_type;

TEST(UtilGeometryMsgsChecks, covarianceChecks) {
    covariance_type covNonSetValues_{-1, 0, 0, 0,  0, 0, 0, -1, 0, 0, 0,  0, 0, 0, -1, 0, 0, 0,
                                     0,  0, 0, -1, 0, 0, 0, 0,  0, 0, -1, 0, 0, 0, 0,  0, 0, -1};

    covariance_type covGroundTruthValues_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    covariance_type covArbitraryDiagValuesValid_{1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0,
                                                 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 6};

    covariance_type covArbitraryValuesValid_{11, 12, 13, 14, 15, 16, 12, 22, 23, 24, 25, 26, 13, 23, 33, 34, 35, 36,
                                             14, 24, 34, 44, 45, 46, 15, 25, 35, 45, 55, 56, 16, 26, 36, 46, 56, 66};

    covariance_type covInvalidUnsymmetric_{1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18,
                                           19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};

    covariance_type covInvalidBelowZero_{-11, 12, 13, 14, 15, 16, 12, 22, 23, 24, 25, 26, 13, 23, 33, 34, 35, 36,
                                         14,  24, 34, 44, 45, 46, 15, 25, 35, 45, 55, 56, 16, 26, 36, 46, 56, -66};

    covariance_type covGroundTruthValuesExceptX1_{-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    covariance_type covGroundTruthValuesExceptX2_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,
                                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0};


    EXPECT_TRUE(covarianceMatrixValid(covNonSetValues_));
    EXPECT_TRUE(covarianceMatrixValid(covGroundTruthValues_));
    EXPECT_TRUE(covarianceMatrixValid(covArbitraryDiagValuesValid_));
    EXPECT_TRUE(covarianceMatrixValid(covArbitraryValuesValid_));
    EXPECT_FALSE(covarianceMatrixValid(covInvalidUnsymmetric_));
    EXPECT_FALSE(covarianceMatrixValid(covInvalidBelowZero_));
    EXPECT_TRUE(covarianceMatrixValid(covGroundTruthValuesExceptX1_));
    EXPECT_TRUE(covarianceMatrixValid(covGroundTruthValuesExceptX2_));

    EXPECT_FALSE(covarianceMatrixIsGroundTruth(covNonSetValues_));
    EXPECT_TRUE(covarianceMatrixIsGroundTruth(covGroundTruthValues_));
    EXPECT_FALSE(covarianceMatrixIsGroundTruth(covArbitraryDiagValuesValid_));
    EXPECT_FALSE(covarianceMatrixIsGroundTruth(covArbitraryValuesValid_));
    EXPECT_FALSE(covarianceMatrixIsGroundTruth(covInvalidUnsymmetric_));
    EXPECT_FALSE(covarianceMatrixIsGroundTruth(covInvalidBelowZero_));
    EXPECT_FALSE(covarianceMatrixIsGroundTruth(covGroundTruthValuesExceptX1_));
    EXPECT_FALSE(covarianceMatrixIsGroundTruth(covGroundTruthValuesExceptX2_));
}

TEST(UtilGeometryMsgsChecks, poseContainsNANs) {

    geometry_msgs::Pose pose = util_geometry_msgs::conversions::toMsg(Eigen::Isometry3d::Identity());
    EXPECT_FALSE(containsNANs(pose));

    pose.position.x = NAN;
    EXPECT_TRUE(containsNANs(pose));
}
