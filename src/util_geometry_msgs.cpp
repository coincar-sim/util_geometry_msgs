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

#include "util_geometry_msgs.hpp"

namespace util_geometry_msgs {

namespace conversions {} // namespace conversions

namespace checks {

bool covarianceMatrixValid(const boost::array<double, 36>& cov) {
    // convert to Eigen::Matrix
    util_eigen_quadratic_matrices::Matrix6d covEigen;
    conversions::boostArrayToEigenMatrix(cov, covEigen);
    return covarianceMatrixValid(covEigen);
}

bool covarianceMatrixIsGroundTruth(const boost::array<double, 36>& cov) {
    if (std::all_of(cov.begin(), cov.end(), [](double d) { return d == 0; })) {
        return true;
    }
    return false;
}

bool topLeft3x3CovarianceMatrixValid(const boost::array<double, 36>& cov) {
    // convert to Eigen::Matrix
    util_eigen_quadratic_matrices::Matrix6d covEigen;
    conversions::boostArrayToEigenMatrix(cov, covEigen);
    return covarianceMatrixValid(covEigen.topLeftCorner(3, 3));
}

bool bottomRight3x3CovarianceMatrixValid(const boost::array<double, 36>& cov) {
    // convert to Eigen::Matrix
    util_eigen_quadratic_matrices::Matrix6d covEigen;
    conversions::boostArrayToEigenMatrix(cov, covEigen);
    return covarianceMatrixValid(covEigen.bottomRightCorner(3, 3));
}


} // namespace checks

namespace transformations {

void transformFromPoses(const geometry_msgs::Pose& oldFrame,
                        const geometry_msgs::Pose& newFrame,
                        Eigen::Isometry3d& transform) {
    Eigen::Isometry3d oldFrameTransformEigen, newFrameTransformEigen;
    // tf2::fromMsg(oldFrame, oldFrameTransformEigen);
    // tf2::fromMsg(newFrame, newFrameTransformEigen);
    conversions::fromMsg(oldFrame, oldFrameTransformEigen);
    conversions::fromMsg(newFrame, newFrameTransformEigen);
    transform = oldFrameTransformEigen * newFrameTransformEigen.inverse();
}

void rereferencePose(const geometry_msgs::Pose& pose,
                     const geometry_msgs::Pose& oldFrame,
                     const geometry_msgs::Pose& newFrame,
                     geometry_msgs::Pose& transformedPose) {

    // get Transform from oldFrame to newFrame
    Eigen::Isometry3d transform;
    transformations::transformFromPoses(oldFrame, newFrame, transform);

    // perform the actual transformation
    Eigen::Isometry3d poseEigen, transformedPoseEigen;
    // tf2::fromMsg(pose, poseEigen);
    conversions::fromMsg(pose, poseEigen);
    transformedPoseEigen = transform * poseEigen;
    transformedPose = tf2::toMsg(transformedPoseEigen);
}

void rereferenceTwist(const geometry_msgs::Twist& twist,
                      const geometry_msgs::Pose& oldFrame,
                      const geometry_msgs::Pose& newFrame,
                      geometry_msgs::Twist& transformedTwist) {

    // get transform from oldFrame to newFrame
    Eigen::Isometry3d transform;
    transformFromPoses(oldFrame, newFrame, transform);

    // Transform the Twist
    Eigen::Vector3d linearTwistEigen, angularTwistEigen, linearTwistEigenTransformed, angularTwistEigenTransformed;
    // tf2::fromMsg(twist.linear, linearTwistEigen);
    // tf2::fromMsg(twist.angular, angularTwistEigen);
    conversions::fromMsg(twist.linear, linearTwistEigen);
    conversions::fromMsg(twist.angular, angularTwistEigen);
    linearTwistEigenTransformed = transform.linear() * linearTwistEigen;
    angularTwistEigenTransformed = transform.linear() * angularTwistEigen;
    // tf2::fromMsg(linearTwistEigen, transformedTwist.linear);
    // tf2::fromMsg(angularTwistEigen, transformedTwist.angular);
    transformedTwist.linear = conversions::toMsg<geometry_msgs::Vector3>(linearTwistEigen);
    transformedTwist.angular = conversions::toMsg<geometry_msgs::Vector3>(angularTwistEigen);
}

void rereferenceCovariance(const geometry_msgs::PoseWithCovariance::_covariance_type& covariance,
                           const geometry_msgs::Pose& oldFrame,
                           const geometry_msgs::Pose& newFrame,
                           geometry_msgs::PoseWithCovariance::_covariance_type& transformedCovariance) {

    // Convert 6DOF Covariance Matrix to Eigen
    util_eigen_quadratic_matrices::Matrix6d covarianceEigen;
    conversions::boostArrayToEigenMatrix(covariance, covarianceEigen);

    // Assuming position and orientation are independent. Hence: Cov({x,y,z};{r,p,y}) = 0
    if (!(covarianceEigen.topRightCorner(3, 3).isApprox(Eigen::Matrix3d::Zero())) &&
        !(covarianceEigen.bottomLeftCorner(3, 3).isApprox(Eigen::Matrix3d::Zero()))) {
        throw std::invalid_argument(
            "Function is only implemented to rerefence covariance matrices with independent orientation and position.");
    }

    // get Transform from oldFrame to newFrame
    Eigen::Isometry3d transform;
    transformFromPoses(oldFrame, newFrame, transform);

    // Build transformation Matrix
    Eigen::Matrix3d rotMatrix3D = transform.rotation();
    util_eigen_quadratic_matrices::Matrix6d rotMatrix6D = util_eigen_quadratic_matrices::Matrix6d::Zero();
    rotMatrix6D.topLeftCorner(3, 3) = rotMatrix3D;
    rotMatrix6D.bottomRightCorner(3, 3) = rotMatrix3D;

    // Rereference Covariance Matrix
    covarianceEigen = rotMatrix6D * covarianceEigen * rotMatrix6D.transpose();

    // convert back to msg
    conversions::eigenMatrixToBoostArray(covarianceEigen, transformedCovariance);
}

void rereferencePoseWithCovariance(const geometry_msgs::PoseWithCovariance& poseWithCovariance,
                                   const geometry_msgs::Pose& oldFrame,
                                   const geometry_msgs::Pose& newFrame,
                                   geometry_msgs::PoseWithCovariance& transformedPoseWithCovariance) {
    rereferencePose(poseWithCovariance.pose, oldFrame, newFrame, transformedPoseWithCovariance.pose);
    rereferenceCovariance(poseWithCovariance.covariance, oldFrame, newFrame, transformedPoseWithCovariance.covariance);
}

void rereferenceTwistWithCovariance(const geometry_msgs::TwistWithCovariance& twistWithCovariance,
                                    const geometry_msgs::Pose& oldFrame,
                                    const geometry_msgs::Pose& newFrame,
                                    geometry_msgs::TwistWithCovariance& transformedTwistWithCovariance) {
    rereferenceTwist(twistWithCovariance.twist, oldFrame, newFrame, transformedTwistWithCovariance.twist);
    rereferenceCovariance(
        twistWithCovariance.covariance, oldFrame, newFrame, transformedTwistWithCovariance.covariance);
}

} // namespace transformations

namespace {
void throwIfScaleInvalid(const double scale) {
    if (scale < 0 || scale > 1) {
        throw std::invalid_argument("Scale not in [0,1] --> no interpolation possible");
    }
}

} // namespace

namespace computations {

geometry_msgs::Pose calcDeltaPose(const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& endPose) {
    // we have: fixed frame <--T_ff_sP-- startPose  AND  fixed frame <--T_ff_eP-- endPose
    // we want: startPose <--T_sP_eP-- endPose ( = deltaPose )
    Eigen::Isometry3d startPoseEigen, endPoseEigen, startPoseEigenInv, deltaPoseEigen;
    geometry_msgs::Pose deltaPoseGeom;

    tf::poseMsgToEigen(startPose, startPoseEigen);
    tf::poseMsgToEigen(endPose, endPoseEigen);

    // invert startPose
    startPoseEigenInv = startPoseEigen.inverse();
    // such that we have: startPose <-- INVERSE(T_ff_sP)-- fixed frame <--T_ff_eP-- endPose

    // now, we concatenate these transformations: // T_sP_eP = T_sP_ff * T_ff_eP
    deltaPoseEigen = startPoseEigenInv * endPoseEigen;
    tf::poseEigenToMsg(deltaPoseEigen, deltaPoseGeom);

    // normalize orientation quaternion
    geometry_msgs::Quaternion deltaQuatGeom = deltaPoseGeom.orientation;
    tf2::Quaternion deltaQuatTf2;
    tf2::fromMsg(deltaQuatGeom, deltaQuatTf2);
    deltaQuatTf2.normalize();
    deltaPoseGeom.orientation = tf2::toMsg(deltaQuatTf2);

    return deltaPoseGeom;
}


geometry_msgs::Point interpolateBetweenPoints(const geometry_msgs::Point& p0,
                                              const geometry_msgs::Point& p1,
                                              const double scale) {
    throwIfScaleInvalid(scale);
    geometry_msgs::Point interpolPosition;

    interpolPosition.x = p0.x + scale * (p1.x - p0.x);
    interpolPosition.y = p0.y + scale * (p1.y - p0.y);
    interpolPosition.z = p0.z + scale * (p1.z - p0.z);

    return interpolPosition;
}

geometry_msgs::Quaternion interpolateBetweenQuaternions(const geometry_msgs::Quaternion& o0,
                                                        const geometry_msgs::Quaternion& o1,
                                                        const double scale) {
    throwIfScaleInvalid(scale);
    geometry_msgs::Quaternion interpolOrientation;

    tf2::Quaternion tfQuat0, tfQuat1, tfQuatInterpol;
    tf2::fromMsg(o0, tfQuat0);
    tf2::fromMsg(o1, tfQuat1);
    tfQuatInterpol = tfQuat0.slerp(tfQuat1, scale).normalized();
    interpolOrientation = tf2::toMsg(tfQuatInterpol);

    return interpolOrientation;
}


geometry_msgs::Pose interpolateBetweenPoses(const geometry_msgs::Pose& p0,
                                            const geometry_msgs::Pose& p1,
                                            const double scale) {
    throwIfScaleInvalid(scale);
    geometry_msgs::Pose interpolPose;

    interpolPose.position = interpolateBetweenPoints(p0.position, p1.position, scale);
    interpolPose.orientation = interpolateBetweenQuaternions(p0.orientation, p1.orientation, scale);

    return interpolPose;
}

geometry_msgs::Pose addDeltaPose(const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& deltaPose) {
    // we have: fixed frame <--T_ff_sP-- startPose <--T_sP_dP-- deltaPose
    // we want: fixed frame <--T_ff_dP-- deltaPose

    Eigen::Isometry3d startPoseEigen, deltaPoseEigen, newPoseEigen;
    geometry_msgs::Pose newPoseGeom;

    tf::poseMsgToEigen(startPose, startPoseEigen);
    tf::poseMsgToEigen(deltaPose, deltaPoseEigen);

    // add deltaPose to startPose
    // startPoseEigen is a transformation from the startPose to the fixed frame T_ff_sP
    // deltaPoseEigen is a transformation from the deltaPose to startPose T_sP_dP
    newPoseEigen = startPoseEigen * deltaPoseEigen;
    // consequently, the new pose is a transformation from the deltaPose to the fixed frame:
    // T_ff_dP = T_ff_sP * T_sP_dP

    tf::poseEigenToMsg(newPoseEigen, newPoseGeom);

    // normalize orientation quaternion
    geometry_msgs::Quaternion newQuatGeom = newPoseGeom.orientation;
    tf2::Quaternion newQuatTf2;
    tf2::fromMsg(newQuatGeom, newQuatTf2);
    newQuatTf2.normalize();
    newPoseGeom.orientation = tf2::toMsg(newQuatTf2);

    return newPoseGeom;
}

geometry_msgs::Pose subtractDeltaPose(const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& deltaPose) {
    // we have: fixed frame <--T_ff_sP-- startPose  AND  deltaPose <--T_dP_sP-- startPose
    // we want: fixed frame <--T_ff_dP-- deltaPose
    Eigen::Isometry3d startPoseEigen, deltaPoseEigen, deltaPoseEigenInv, newPoseEigen;
    geometry_msgs::Pose newPoseGeom;

    tf::poseMsgToEigen(startPose, startPoseEigen);
    tf::poseMsgToEigen(deltaPose, deltaPoseEigen);

    // invert deltaPose
    deltaPoseEigenInv = deltaPoseEigen.inverse();
    // such that we have: fixed frame <--T_ff_sP-- startPose <--INVERSE(T_dP_sP)-- deltaPose

    // now, we concatenate these transformations as in addDeltaPose
    newPoseEigen = startPoseEigen * deltaPoseEigenInv;
    tf::poseEigenToMsg(newPoseEigen, newPoseGeom);

    // normalize orientation quaternion
    geometry_msgs::Quaternion newQuatGeom = newPoseGeom.orientation;
    tf2::Quaternion newQuatTf2;
    tf2::fromMsg(newQuatGeom, newQuatTf2);
    newQuatTf2.normalize();
    newPoseGeom.orientation = tf2::toMsg(newQuatTf2);

    return newPoseGeom;
}


} // namespace computations


} // namespace util_geometry_msgs
