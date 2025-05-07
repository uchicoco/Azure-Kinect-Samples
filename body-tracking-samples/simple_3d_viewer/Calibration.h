#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Structure to store rigid body transformations (rotation + translation)
 */
struct Transform {
    cv::Mat R; // Rotation (3x3)
    cv::Mat t; // Translation (3x1)

    /**
     * Function to convert a point using this transformation
     * @param p The input 3D point to transform
     * @return The transformed 3D point
     */
    cv::Point3f apply(const cv::Point3f& p) const;
};

/**
 * Structure to return the calibration result and matrix
 */
struct CalibrationResult {
    bool success;
    std::vector<Transform> transforms;
};

/**
 * Estimate rigid body transformation (Affine3D + RANSAC) and separate into rotation + translation
 * 
 * @param subPoints The 3D points from the sub camera
 * @param mainPoints The corresponding 3D points from the main camera
 * @param outTransform The resulting transformation (output parameter)
 * @return True if transformation was successfully estimated, false otherwise
 */
bool estimateRigidTransformRANSAC(
    const std::vector<cv::Point3f>& subPoints,
    const std::vector<cv::Point3f>& mainPoints,
    Transform& outTransform
);

/**
 * Calibration of 4 cameras (3 sub cameras + 1 main camera)
 * 
 * @param subPoints1 3D points from first sub camera
 * @param subPoints2 3D points from second sub camera
 * @param subPoints3 3D points from third sub camera
 * @param mainPoints 3D points from main camera
 * @return CalibrationResult containing success status and transformations
 */
CalibrationResult calibration(
    const std::vector<cv::Point3f>& subPoints1,
    const std::vector<cv::Point3f>& subPoints2,
    const std::vector<cv::Point3f>& subPoints3,
    const std::vector<cv::Point3f>& mainPoints
);

/**
 * Example usage of the calibration functionality
 * 
 * @return Example status code
 */
int usage();
