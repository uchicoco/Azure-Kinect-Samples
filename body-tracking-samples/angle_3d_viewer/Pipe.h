#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <k4a/k4a.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/**
 * Initialize the named pipe for angle data communication
 * @return true if successful, false otherwise
 */
bool InitializeNamedPipe();

/**
 * Open the named pipe for writing
 * @return true if successful, false if no reader available
 */
bool OpenNamedPipe();

/**
 * Send angle data through the named pipe
 * @param body_id ID of the tracked body
 * @param right_arm_angle Right arm angle in degrees
 * @param left_arm_angle Left arm angle in degrees
 * @param legs_angle Legs angle in degrees
 * @param timestamp Timestamp of the measurement
 */
void SendAngleDataToPipe(uint32_t body_id, double right_arm_angle, double left_arm_angle, double legs_angle, uint64_t timestamp);

/**
 * Clean up and close the named pipe
 */
void CleanupNamedPipe();
