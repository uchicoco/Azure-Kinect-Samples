#pragma once
// SaveCSV.h
#pragma once

#include <iostream>
#include <stdexcept>
#include <BodyTrackingHelpers.h>

/**
 * @brief Function to save joint positions to a CSV file.
 * 
 * @param body Body data
 * @param csvFile CSV file stream to write to
 * @param frameCount Timestamp of the frame
 */
void SaveJointPositionsToCSV(const k4abt_body_t& body, std::ofstream& csvFile, uint64_t timestamp);

/**
 * @brief Gets the current timestamp in microseconds.
 *
 * This function returns a high-resolution timestamp measured in microseconds.
 * - On Windows, it uses QueryPerformanceCounter and QueryPerformanceFrequency.
 * - On Linux/Unix, it uses clock_gettime with CLOCK_MONOTONIC.
 *
 * @return uint64_t The current timestamp in microseconds.
 */
uint64_t GetTimestamp();
