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
 * @param frameCount Frame count
 */
void SaveJointPositionsToCSV(const k4abt_body_t& body, std::ofstream& csvFile, uint64_t frameCount);
