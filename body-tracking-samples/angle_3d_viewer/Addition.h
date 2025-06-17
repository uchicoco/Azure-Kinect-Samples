#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <BodyTrackingHelpers.h>

/**
 * @brief Function to save joint positions to a CSV file.
 * 
 * @param body Body data
 * @param csvFile CSV file stream to write to
 * @param timestamp Timestamp of the frame
 * @throws std::runtime_error if file operations fail
 */
void SaveJointPositionsToCSV(const k4abt_body_t& body, std::ofstream& csvFile, uint64_t timestamp);

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
 */
void SaveColorImage(const k4a_image_t& colorImage, const std::string& folderPath, uint64_t timestamp, uint64_t frameCount);