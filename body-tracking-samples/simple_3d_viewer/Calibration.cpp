#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

// Structure to store rigid body transformations (rotation + translation)
struct Transform {
    cv::Mat R; // Rotation (3x3)
    cv::Mat t; // Translation (3x1)

    // Function to convert a point
    cv::Point3f apply(const cv::Point3f& p) const {
        cv::Mat pt = (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);
        cv::Mat result = R * pt + t;
        return cv::Point3f(result.at<double>(0), result.at<double>(1), result.at<double>(2));
    }
};

// Estimate rigid body transformation (Affine3D + RANSAC) and separate into rotation + translation
bool estimateRigidTransformRANSAC(
    const std::vector<cv::Point3f>& subPoints,
    const std::vector<cv::Point3f>& mainPoints,
    Transform& outTransform
) {
    if (subPoints.size() < 3 || mainPoints.size() < 3 || subPoints.size() != mainPoints.size()) {
        std::cerr << "Need at least 3 matching points of same length!" << std::endl;
        return false;
    }

    cv::Mat affine;  // 3x4 Affine Matrix
    std::vector<uchar> inliers;

    double ransacThreshold = 3.0;   // Threshold value to be considered an outlier (distance)
    double confidence = 0.99;       // RANSAC confidence level

    int success = cv::estimateAffine3D(subPoints, mainPoints, affine, inliers, ransacThreshold, confidence);

    if (!success) {
        std::cerr << "estimateAffine3D failed!" << std::endl;
        return false;
    }

    // Decomposition
    outTransform.R = affine(cv::Range(0, 3), cv::Range(0, 3)).clone();
    outTransform.R.convertTo(outTransform.R, CV_64F); // Transform to double
    outTransform.t = affine(cv::Range(0, 3), cv::Range(3, 4)).clone();
    outTransform.t.convertTo(outTransform.t, CV_64F);

    return true;
}

// Structure to return the calibration result and matrix
struct CalibrationResult {
    bool success;
    std::vector<Transform> transforms;
};

// Calibration of 4 cameras (3 sub cameras + 1 main camera)
CalibrationResult calibration(
    const std::vector<cv::Point3f>& subPoints1,
    const std::vector<cv::Point3f>& subPoints2,
    const std::vector<cv::Point3f>& subPoints3,
    const std::vector<cv::Point3f>& mainPoints
) {
    CalibrationResult result;
	result.transforms.resize(3); // Store 3 transforms for 3 sub cameras

	// Calculate the rigid body transformations for each sub camera
    bool success1 = estimateRigidTransformRANSAC(subPoints1, mainPoints, result.transforms[0]);
    bool success2 = estimateRigidTransformRANSAC(subPoints2, mainPoints, result.transforms[1]);
    bool success3 = estimateRigidTransformRANSAC(subPoints3, mainPoints, result.transforms[2]);

	// Check if all transformations were successful
    result.success = success1 && success2 && success3;

    return result;
}

void usage() {
    // ==== 例: SubカメラとMainカメラでの対応点 ====
    std::vector<cv::Point3f> subPoints1 = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {1.0f, 1.0f, 0.0f},
        {0.5f, 0.5f, 1.0f}
    };

    std::vector<cv::Point3f> subPoints2 = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {1.0f, 1.0f, 0.0f},
        {0.5f, 0.5f, 1.0f}
    };

    std::vector<cv::Point3f> subPoints3 = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {1.0f, 1.0f, 0.0f},
        {0.5f, 0.5f, 1.0f}
    };

    std::vector<cv::Point3f> mainPoints = {
        {0.1f, 0.2f, 0.0f},
        {1.1f, 0.2f, 0.0f},
        {0.1f, 1.2f, 0.0f},
        {1.1f, 1.2f, 0.0f},
        {0.6f, 0.7f, 1.0f}
    };

    // ==== キャリブレーション（変換行列を求める） ====
    CalibrationResult calibResult = calibration(subPoints1, subPoints2, subPoints3, mainPoints);

    if (calibResult.success) {
        std::cout << "Calibration success!" << std::endl;

        // 変換行列を使用（例：最初のサブカメラのポイントを変換）
        cv::Point3f point = { 1.0f, 2.0f, 3.0f };
        cv::Point3f transformedPoint = calibResult.transforms[0].apply(point);

        std::cout << "変換後のポイント: " << transformedPoint.x << ", "
            << transformedPoint.y << ", " << transformedPoint.z << std::endl;
    }
    else {
        std::cout << "キャリブレーション失敗" << std::endl;
    }
}
