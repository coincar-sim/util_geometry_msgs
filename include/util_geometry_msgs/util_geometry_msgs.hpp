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

#pragma once

#include <boost/array.hpp>

#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <util_eigen_geometry/util_eigen_geometry.hpp>
#include <util_eigen_geometry/util_eigen_quadratic_matrices.hpp>

namespace util_geometry_msgs {

// Name cannot be changed due to backwards compatibility.
// NOLINTNEXTLINE(readability-identifier-naming)
using covariance_t = boost::array<double, 36>;

namespace conversions {

/** \brief Convert a Pose message transform type to a Eigen Isometry3d.
 * This function is a supplement to the "fromMsg(const geometry_msgs::Pose& msg, Eigen::Affine3d& out)" conversion
 * defined in <tf2_eigen/tf2_eigen.h>.
 * \param msg The Pose message to convert.
 * \param out The pose converted to a Eigen Isometry3d.
 */
inline void fromMsg(const geometry_msgs::Pose& msg, Eigen::Isometry3d& out) {
    // see tf2_eigen: void 	tf2::fromMsg (const geometry_msgs::Pose &msg, Eigen::Affine3d &out)
    out = Eigen::Isometry3d(
        Eigen::Translation3d(msg.position.x, msg.position.y, msg.position.z) *
        Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z));
}

inline void fromMsg(const geometry_msgs::Pose& msg, Eigen::Vector3d& out) {

    out.x() = msg.position.x;
    out.y() = msg.position.y;
    out.z() = msg.position.z;
}


/** \brief Convert a Eigen Isometry3d transform type to a Pose message.
 * This function is a supplement to the "geometry_msgs::Pose toMsg(const Eigen::Affine3d& in)" conversion defined
 * in <tf2_eigen/tf2_eigen.h>
 * \param in The Eigen Isometry3d to convert.
 * \return The Eigen transform converted to a Pose message.
 */
inline geometry_msgs::Pose toMsg(const Eigen::Isometry3d& in) {
    // see tf2_eigen: geometry_msgs::Pose 	tf2::toMsg (const Eigen::Affine3d &in)
    geometry_msgs::Pose msg;
    msg.position.x = in.translation().x();
    msg.position.y = in.translation().y();
    msg.position.z = in.translation().z();
    Eigen::Quaterniond orientation = static_cast<Eigen::Quaterniond>(in.linear());
    msg.orientation.x = orientation.x();
    msg.orientation.y = orientation.y();
    msg.orientation.z = orientation.z();
    msg.orientation.w = orientation.w();
    if (msg.orientation.w < 0) {
        msg.orientation.x = -1 * msg.orientation.x;
        msg.orientation.y = -1 * msg.orientation.y;
        msg.orientation.z = -1 * msg.orientation.z;
        msg.orientation.w = -1 * msg.orientation.w;
    }
    return msg;
}


/** \brief Convert a 3d-Vector-Type message to a Eigen Vector3d.
 * This function is a supplement to the "void fromMsg(const geometry_msgs::Point& msg, Eigen::Vector3d& out)" conversion
 * defined in <tf2_eigen/tf2_eigen.h>.
 * It extends the use to other 3dimensional Vector types such as geometry_msgs:Vector3D
 * \param msg The message to convert (3dimensional vector type with x,y,z naming scheme).
 * \param out The message converted to a Eigen Vector3d.
 */
template <typename Type3D>
inline void fromMsg(const Type3D& msg, Eigen::Vector3d& out) {
    out.x() = msg.x;
    out.y() = msg.y;
    out.z() = msg.z;
}

/** \brief Convert a Eigen Vector3d to a 3d-Vector-Type message.
 * This function is a supplement to the geometry_msgs::Point toMsg(Eigen::Vector3d& out) conversion
 * defined in <tf2_eigen/tf2_eigen.h>.
 * It extends the use to other 3dimensional Vector types such as geometry_msgs:Vector3D
 * \param Eigen Vector3d to convert.
 * \return The vector converted to a message (3dimensional vector type with x,y,z naming scheme).
 */
template <typename Type3D>
Type3D toMsg(const Eigen::Vector3d& in) {
    Type3D msg;
    msg.x = in.x();
    msg.y = in.y();
    msg.z = in.z();
    return msg;
}

/** @brief Transform matrix in boost:array storage to Eigen::Matrix storage
 * @param[in] Matrix stored as boost::array
 * @param[out] Matrix stored as Eigen::matrix
 * @param[in, optional] input boost:array is using row-major-representation(true) or column-major-representation(false)
 * @return void */
template <typename Scalar, int Size, int MajorOption>
inline void boostArrayToEigenMatrix(const boost::array<Scalar, Size * Size>& boostArray,
                                    Eigen::Matrix<Scalar, Size, Size, MajorOption>& eigenMatrix,
                                    bool boostArrayIsRowMajor = true) {
    if ((boostArrayIsRowMajor && MajorOption == Eigen::RowMajor) ||
        (!boostArrayIsRowMajor && MajorOption == Eigen::ColMajor)) {
        // input as well as output using row-major-representation or column-major-representation
        for (int index = 0; index < Size * Size; index++) {
            eigenMatrix(index) = boostArray[index];
        }
    } else if (!boostArrayIsRowMajor && MajorOption == Eigen::RowMajor) {
        // input is using Column Major representation output is using row-major-representation
        Eigen::Matrix<Scalar, Size, Size, Eigen::ColMajor> tempMatrix;
        for (int index = 0; index < Size * Size; index++) {
            tempMatrix(index) = boostArray[index];
        }
        eigenMatrix = tempMatrix;
    } else if (boostArrayIsRowMajor && MajorOption == Eigen::ColMajor) {
        // input is using Row-Major representation output is using column-major-representation
        Eigen::Matrix<Scalar, Size, Size, Eigen::RowMajor> tempMatrix;
        for (int index = 0; index < Size * Size; index++) {
            tempMatrix(index) = boostArray[index];
        }
        eigenMatrix = tempMatrix;
    }
}

/** @brief Transform matrix in boost:array storage to Eigen::Matrix storage
 * @param[in] Matrix stored as Eigen::matrix
 * @param[out] Matrix stored as boost::array
 * @param[in, optional] output boost:array is using row-major-representation(true) or column-major-representation(false)
 * @return void */
template <typename Scalar, int Size, int MajorOption>
inline void eigenMatrixToBoostArray(const Eigen::Matrix<Scalar, Size, Size, MajorOption>& eigenMatrix,
                                    boost::array<Scalar, Size * Size>& boostArray,
                                    bool boostArrayIsRowMajor = true) {
    if ((boostArrayIsRowMajor && MajorOption == Eigen::RowMajor) ||
        (!boostArrayIsRowMajor && MajorOption == Eigen::ColMajor)) {
        // input as well as output using row-major-representation or column-major-representation
        for (int index = 0; index < Size * Size; index++) {
            boostArray[index] = eigenMatrix(index);
        }
    } else if (!boostArrayIsRowMajor && MajorOption == Eigen::RowMajor) {
        // output is using Column Major representation, input is using row-major-representation
        Eigen::Matrix<Scalar, Size, Size, Eigen::ColMajor> tempMatrix = eigenMatrix;
        for (int index = 0; index < Size * Size; index++) {
            boostArray[index] = tempMatrix(index);
        }
    } else if (boostArrayIsRowMajor && MajorOption == Eigen::ColMajor) {
        // output is using Row-Major representation, input is using column-major-representation
        Eigen::Matrix<Scalar, Size, Size, Eigen::RowMajor> tempMatrix = eigenMatrix;
        for (int index = 0; index < Size * Size; index++) {
            boostArray[index] = tempMatrix(index);
        }
    }
}

inline double yawFromPose(const geometry_msgs::Pose& pose) {
    Eigen::Isometry3d eigenPose;
    tf2::fromMsg(pose, eigenPose);
    return util_eigen_geometry::yawFromIsometry3d(eigenPose);
}

inline double yawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation = quat;
    return yawFromPose(pose);
}

inline geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    geometry_msgs::Quaternion quat = tf2::toMsg(q);
    return quat;
}

inline geometry_msgs::Transform transformFromPose(const geometry_msgs::Pose& pose) {
    geometry_msgs::Transform transform;

    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation.x = pose.orientation.x;
    transform.rotation.y = pose.orientation.y;
    transform.rotation.z = pose.orientation.z;
    transform.rotation.w = pose.orientation.w;

    return transform;
}

} // namespace conversions

namespace checks {

inline bool containsNANs(const geometry_msgs::Pose& pose) {
    return (std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z) ||
            std::isnan(pose.orientation.x) || std::isnan(pose.orientation.y) || std::isnan(pose.orientation.z) ||
            std::isnan(pose.orientation.w));
}

template <typename QuaternionType>
inline bool quaternionNormalized(const QuaternionType& quaternion) {
    return (std::abs((quaternion.w * quaternion.w + quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                      quaternion.z * quaternion.z) -
                     1.0f) < 10e-3);
}


/** @brief Checks if a matrix is a valid covariance matrix (positive (semi) definite and symmetric)
 * Only the covariance matrix of the valid entries
 * (as defined following ROS-Convention: invalid values are marked with -1 on the diagonal of the covariance matrix)
 * is checked.
 * E.g. consider the Covariance Matrix for a 6D-pose (3D-position + 3D-orientation) and all values of the
 * orientation
 * are marked as not defined by "-1" on the diagonal.
 * Then the 6x6 covariance matrix for position+orientation is reduced in a first step to a position only 3x3
 * covariance
 * matrix and then this reduced matrix is checked for validity.
 * Entries that are marked with "-1" on the diagonal are excluded from the check.
 * @param[in] quadratic input Matrix
 * @return true if the covariance matrix is valid for all values that are not marked with "-1" on the diagonal or
 * false
 * otherwise.*/
template <typename Derived>
bool covarianceMatrixValid(const Eigen::MatrixBase<Derived>& covarianceMatrixInput) {
    // Check which values (not variance/variance values) are marked as not defined or unreliable per ROS-definition
    // (-1
    // on diagonal of covariance Matrix)
    // and Shrink temporary working copy of covariance matrix to covariance matrix for valid values
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> covarianceMatrix = covarianceMatrixInput;
    Eigen::VectorXd diag = covarianceMatrix.diagonal();
    for (int i = static_cast<int>(diag.size()) - 1; i >= 0; i--) {
        if (std::abs(diag(i) + 1.) < 10.e-9) {
            covarianceMatrix = util_eigen_quadratic_matrices::removeRowCol(covarianceMatrix, i);
        }
    }
    // Check shrinked covariance Matrix for validity
    // special case for 0D-zero-elements Matrix. This means the covariance matrix is theoratically valid, but for
    // zero elements.
    if (covarianceMatrix.rows() == 0) {
        return true;
    }
    return (util_eigen_quadratic_matrices::isSymmetric(covarianceMatrix) &&
            util_eigen_quadratic_matrices::isPosSemiDefinit(covarianceMatrix));
}

/** @brief Checks if a matrix is a valid covariance matrix (positive (semi) definite and symmetric)
 * Only the covariance matrix of the valid entries
 * (as defined following ROS-Convention: invalid values are marked with -1 on the diagonal of the covariance matrix)
 * is checked.
 * E.g. consider the Covariance Matrix for a 6D-pose (3D-position + 3D-orientation) and all values of the
 * orientation
 * are marked as not defined by "-1" on the diagonal.
 * Then the 6x6 covariance matrix for position+orientation is reduced in a first step to a position only 3x3
 * covariance
 * matrix and then this reduced matrix is checked for validity.
 * Entries that are marked with "-1" on the diagonal are excluded from the check.
 * @param[in] 6x6 quadratic input Matrix as boost::array<double, 36> (e.g. from geometry_msgs::PoseWithCovariance)
 * @return true if the covariance matrix is valid for all values that are not marked with "-1" on the diagonal or
 * false
 * otherwise.*/
bool covarianceMatrixValid(const boost::array<double, 36>& cov);

/** @brief Checks if a matrix is a valid covariance matrix for ground truth values.
 *   It is checked if all entries in the covariance matrix are zero (0)
 * @param[in] 6x6 quadratic input Matrix as boost::array<double, 36> (e.g. from geometry_msgs::PoseWithCovariance)
 * @return true if all entries in the covariance matrix are zero (0), false otherwise.
 */
bool covarianceMatrixIsGroundTruth(const boost::array<double, 36>& cov);

/** @brief Checks if the top-left quadratic 3x3 matrix is a valid covariance matrix (positive (semi) definite and
 * symmetric)
 * E.g. use this to check only the covariance matrix for the position in a pose (position + orientation)
 * Only the covariance matrix of the valid entries
 * (as defined following ROS-Convention: invalid values are marked with -1 on the diagonal of the covariance matrix)
 * is checked.
 * E.g. consider the Covariance Matrix for a 3D-position (X,Y,Z) and the Z-Value is marked unreliable then
 * then only the 2x2 covariance matrix for {X,Y} is reduced in a first step to a 2x2 covariance
 * matrix and then this reduced matrix is checked for validity.
 * Entries that are marked with "-1" on the diagonal are excluded from the check.
 * @param[in] 6x6 quadratic input Matrix as boost::array<double, 36> (e.g. from geometry_msgs::PoseWithCovariance)
 * @return true if the Top-Left 3x3 covariance matrix is valid for all values that are not marked with "-1" on the
 * diagonal or false
 * otherwise.*/
bool topLeft3x3CovarianceMatrixValid(const boost::array<double, 36>& cov);

/** @brief Checks if the bottom-right quadratic 3x3 matrix is a valid covariance matrix (positive (semi) definite
 * and
 * symmetric)
 * E.g. use this to check only the covariance matrix for the orientation in a pose (position + orientation)
 * Only the covariance matrix of the valid entries
 * (as defined following ROS-Convention: invalid values are marked with -1 on the diagonal of the covariance matrix)
 * is checked.
 * E.g. consider the Covariance Matrix for a 3D-orientation (X,Y,Z) and the Z-Value is marked unreliable then
 * then only the 2x2 covariance matrix for {X,Y} is reduced in a first step to a 2x2 covariance
 * matrix and then this reduced matrix is checked for validity.
 * Entries that are marked with "-1" on the diagonal are excluded from the check.
 * @param[in] 6x6 quadratic input Matrix as boost::array<double, 36> (e.g. from geometry_msgs::PoseWithCovariance)
 * @return true if the Bootm-Right 3x3 covariance matrix is valid for all values that are not marked with "-1" on
 * the
 * diagonal or false
 * otherwise.*/
bool bottomRight3x3CovarianceMatrixValid(const boost::array<double, 36>& cov);


// Covariance Matrix to mark all values as undefined or unreliable.
// clang-format off
// Cannot change name due to backwards compatibility.
// NOLINTNEXTLINE(readability-identifier-naming)
static constexpr boost::array<double, 36> covarianceUnkownValues = {{
    -1, 0, 0, 0, 0, 0,
    0, -1, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0,
    0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, -1, 0,
    0, 0, 0, 0, 0, -1}};
// clang-format on

// Covariance Matrix to mark all values as ground truth values.
// Cannot change name due to backwards compatibility.
// NOLINTNEXTLINE(readability-identifier-naming)
static constexpr boost::array<double, 36> covarianceGroundTruthValues = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

} // namespace checks

namespace transformations {

/** @brief Interpret Poses as Frames and get the Transformation from OldFrame to a NewFrame.
 * @param[in] oldFrame from which the transformation originates (geometry_msgs::Pose)
 * @param[in] newFrame into which the transformation is specified (geometry_msgs::Pose)
 * @param[out] The Transform from oldFrame to newFrame (Eigen::Isometry3d)
 * @return void */
void transformFromPoses(const geometry_msgs::Pose& oldFrame,
                        const geometry_msgs::Pose& newFrame,
                        Eigen::Isometry3d& transform);

/** @brief Rereference a Pose from oldFrame to newFrame.
 * It is a coordinate transformation from the coordinate System defined by oldFrame to the coordinate
 * System defined by newFrame.
 * @param[in] pose: Pose defined in oldFrame
 * @param[in] oldFrame defined by an origin (oldFrame.position) and orientation (oldFrame.orientation)
 * @param[in] newFrame defined by an origin (newFrame.position) and orientation (newFrame.orientation)
 * @param[out] transformedPose: Pose rereferenced to newFrame
 * @return void */
void rereferencePose(const geometry_msgs::Pose& pose,
                     const geometry_msgs::Pose& oldFrame,
                     const geometry_msgs::Pose& newFrame,
                     geometry_msgs::Pose& transformedPose);

/** @brief Rereference a Twist from oldFrame to newFrame.
 * It is a coordinate transformation from the coordinate System defined by oldFrame to the coordinate
 * System defined by newFrame.
 * @param[in] twist: Twist defined in oldFrame
 * @param[in] oldFrame defined by an origin (oldFrame.position) and orientation (oldFrame.orientation)
 * @param[in] newFrame defined by an origin (newFrame.position) and orientation (newFrame.orientation)
 * @param[out] transformedtwist: Twist rereferenced to newFrame
 * @return void */
void rereferenceTwist(const geometry_msgs::Twist& twist,
                      const geometry_msgs::Pose& oldFrame,
                      const geometry_msgs::Pose& newFrame,
                      geometry_msgs::Twist& transformedTwist);

/** @brief Rereference a covariance matrix from oldFrame to newFrame.
 * So basically it is a coordinate transformation from the coordinate System defined by oldFrame to the coordinate
 * System defined by newFrame.
 * Assuming position and orientation are independent. Hence: Cov({x,y,z};{roll,pitch,yaw}) = 0
 * If this condition is not met std::invalid_argument is thrown.
 * @param[in] covariance: 6DOF covariance matrix defined in oldFrame
 * @param[in] oldFrame defined by an origin (oldFrame.position) and orientation (oldFrame.orientation)
 * @param[in] newFrame defined by an origin (newFrame.position) and orientation (newFrame.orientation)
 * @param[out] transformedCovariance: 6DOF covariance matrix rereferenced to newFrame
 * @return void */
void rereferenceCovariance(const geometry_msgs::PoseWithCovariance::_covariance_type& covariance,
                           const geometry_msgs::Pose& oldFrame,
                           const geometry_msgs::Pose& newFrame,
                           geometry_msgs::PoseWithCovariance::_covariance_type& transformedCovariance);

/** @brief Rereference a Pose and its covariance from oldFrame to newFrame.
 * It is a coordinate transformation from the coordinate system defined by oldFrame to the coordinate
 * system defined by newFrame.
 * @param[in] poseWithCovariance: Pose and covariance matrix defined in oldFrame
 * @param[in] oldFrame defined by an origin (oldFrame.position) and orientation (oldFrame.orientation)
 * @param[in] newFrame defined by an origin (newFrame.position) and orientation (newFrame.orientation)
 * @param[out] transformedPoseWithCovariance: Pose and covariancte matrix rereferenced to newFrame
 * @return void */
void rereferencePoseWithCovariance(const geometry_msgs::PoseWithCovariance& poseWithCovariance,
                                   const geometry_msgs::Pose& oldFrame,
                                   const geometry_msgs::Pose& newFrame,
                                   geometry_msgs::PoseWithCovariance& transformedPoseWithCovariance);

/** @brief Rereference a Twist and its covariance from oldFrame to newFrame.
 * So basically it is a coordinate transformation from the coordinate System defined by oldFrame to the coordinate
 * System defined by newFrame.
 * @param[in] twistWithCovariance: Twist and covariance defined in oldFrame
 * @param[in] oldFrame defined by an origin (oldFrame.position) and orientation (oldFrame.orientation)
 * @param[in] newFrame defined by an origin (newFrame.position) and orientation (newFrame.orientation)
 * @param[out] transformedTwistWithCovariance: Twist and covariancerereferenced to newFrame
 * @return void */
void rereferenceTwistWithCovariance(const geometry_msgs::TwistWithCovariance& twistWithCovariance,
                                    const geometry_msgs::Pose& oldFrame,
                                    const geometry_msgs::Pose& newFrame,
                                    geometry_msgs::TwistWithCovariance& transformedTwistWithCovariance);


} // namespace transformations

namespace computations {

geometry_msgs::Pose calcDeltaPose(const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& endPose);

geometry_msgs::Point interpolateBetweenPoints(const geometry_msgs::Point& p0,
                                              const geometry_msgs::Point& p1,
                                              double scale);

geometry_msgs::Quaternion interpolateBetweenQuaternions(const geometry_msgs::Quaternion& o0,
                                                        const geometry_msgs::Quaternion& o1,
                                                        double scale);

geometry_msgs::Pose interpolateBetweenPoses(const geometry_msgs::Pose& p0, const geometry_msgs::Pose& p1, double scale);

geometry_msgs::Pose addDeltaPose(const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& deltaPose);

geometry_msgs::Pose subtractDeltaPose(const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& deltaPose);


} // namespace computations

} // namespace util_geometry_msgs
