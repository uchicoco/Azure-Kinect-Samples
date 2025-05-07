#pragma once
// No changes needed in this file for now.
#include <iostream>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <opencv2/opencv.hpp>
#include <BodyTrackingHelpers.h>
#include "Calibration.h"

/**
 * @brief Function to save multiple bodies' joint positions to a CSV file in a batch.
 * 
 * This is more efficient than calling SaveJointPositionsToCSV multiple times
 * as it reduces file access operations and locking overhead.
 * 
 * @param bodies Vector of body data
 * @param csvFile CSV file stream to write to
 * @param timestamp Timestamp of the frame
 * @throws std::runtime_error if file operations fail
 */
void SaveMultipleBodiesToCSV(const std::vector<k4abt_body_t>& bodies, std::ofstream& csvFile, uint64_t timestamp);

/**
 * @brief Gets the current timestamp in microseconds.
 *
 * This function returns a high-resolution timestamp measured in microseconds.
 * - On Windows, it uses QueryPerformanceCounter and QueryPerformanceFrequency.
 * - On Linux/Unix, it uses clock_gettime with CLOCK_MONOTONIC.
 *
 * @return uint64_t The current timestamp in microseconds, or 0 on error.
 */
uint64_t GetTimestamp();

/**
 * @brief Function to save a color image to disk.
 *
 * @param colorImage Color image to save
 * @param folderPath Folder path to save the image
 * @param timestamp Timestamp of the frame
 * @param frameCount Frame count of the image
 * @throws std::runtime_error if file operations fail
 */
void SaveColorImage(const k4a_image_t& colorImage, const std::string& folderPath, uint64_t timestamp, uint64_t frameCount);

/**
 * @brief Function to transform body data into a vector of 3D points.
 * 
 * @param body Body data to convert
 * @return std::vector<cv::Point3f> Vector of 3D points representing joint positions
 */
std::vector<cv::Point3f> ConvertBodyToPoints(const k4abt_body_t& body);

/**
 * @brief Calibrates cameras by calculating transformations between 3 sub cameras and 1 main camera.
 * 
 * @param body1 Body data from first sub camera
 * @param body2 Body data from second sub camera
 * @param body3 Body data from third sub camera
 * @param body_m Body data from main camera
 * @return std::vector<Transform> Vector of transformation matrices for each sub camera
 * @throws std::runtime_error if calibration fails
 */
std::vector<Transform> CalibrateCameras(const k4abt_body_t& body1, const k4abt_body_t& body2, const k4abt_body_t& body3, const k4abt_body_t& body_m);

/**
 * @brief Applies a transformation matrix to all joint positions in a body.
 * 
 * This function transforms all joint positions in the input body by applying the provided
 * transformation matrix. The body's ID is preserved, but all joint positions are modified
 * according to the transformation.
 * 
 * @param body The body whose joint positions will be transformed
 * @param transform The transformation matrix to apply
 * @return k4abt_body_t A new body with transformed joint positions
 */
k4abt_body_t ApplyTransformToBody(const k4abt_body_t& body, const Transform& transform);

/**
 * @brief Applies transformation matrices to multiple bodies.
 * 
 * This function transforms each body in the input vector using the specified
 * transformation matrix. Useful for converting coordinates from one camera
 * reference frame to another.
 * 
 * @param bodies Vector of bodies to transform
 * @param transform The transformation matrix to apply to all bodies
 * @return std::vector<k4abt_body_t> Vector of transformed bodies
 */
std::vector<k4abt_body_t> ApplyTransformToBodies(const std::vector<k4abt_body_t>& bodies, const Transform& transform);

/**
 * Applies a transformation to a vector of bodies and returns the transformed bodies.
 * @param bodies The vector of bodies to transform.
 * @param transform The transformation to apply.
 * @return A vector of transformed bodies.
 */
std::vector<k4abt_body_t> ApplyTransformToBodies(const std::vector<k4abt_body_t>& bodies, const Transform& transform);

/**
 * @brief Determines if two bodies represent the same person based on joint positions.
 *
 * @param body1 First body to compare
 * @param body2 Second body to compare
 * @param distanceThresholdMm Maximum average distance in mm between joints to consider bodies as the same person (default: 100mm)
 * @return true if bodies likely represent the same person
 * @return false if bodies are likely different people or comparison failed
 */
bool IsSameBody(const k4abt_body_t& body1, const k4abt_body_t& body2, float distanceThresholdMm);
// ...existing code...

/**
 * Merges and transforms multiple sets of bodies into a single unified set.
 * @param bodies1 The first set of bodies.
 * @param bodies2 The second set of bodies.
 * @param bodies3 The third set of bodies.
 * @param bodies4 The fourth set of bodies.
 * @param transform1 The transformation to apply to bodies1.
 * @param transform2 The transformation to apply to bodies2.
 * @param transform3 The transformation to apply to bodies3.
 * @return A unified vector of bodies with unique IDs and confidence-weighted joint positions.
 */
std::vector<k4abt_body_t> MergeAndTransformBodies(
    const std::vector<k4abt_body_t>& bodies1,
    const std::vector<k4abt_body_t>& bodies2,
    const std::vector<k4abt_body_t>& bodies3,
    const std::vector<k4abt_body_t>& bodies4,
    const Transform& transform1,
    const Transform& transform2,
    const Transform& transform3);

// ...existing code...
/**
 * Merges and transforms multiple sets of bodies into a single unified set.
 * @param bodies1 The first set of bodies.
 * @param bodies2 The second set of bodies.
 * @param bodies3 The third set of bodies.
 * @param bodies4 The fourth set of bodies.
 * @param transform1 The transformation to apply to bodies1.
 * @param transform2 The transformation to apply to bodies2.
 * @param transform3 The transformation to apply to bodies3.
 * @return A unified vector of bodies with unique IDs and confidence-weighted joint positions.
 */
std::vector<k4abt_body_t> MergeAndTransformBodies(
    const std::vector<k4abt_body_t>& bodies1,
    const std::vector<k4abt_body_t>& bodies2,
    const std::vector<k4abt_body_t>& bodies3,
    const std::vector<k4abt_body_t>& bodies4,
    const Transform& transform1,
    const Transform& transform2,
    const Transform& transform3);
