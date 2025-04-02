#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <k4a/k4a.h>

#include <BodyTrackingHelpers.h>
#include <Eigen/Dense>
#ifdef _WIN32
	#include <Windows.h>
#else
	#include <unistd.h>
	#include <sys/time.h>
#endif
#include "Addition.h"
#include "AngleCalculator.h"

// Mutex for file access synchronization
static std::mutex g_fileMutex;

void SaveMultipleBodiesToCSV(const std::vector<k4abt_body_t>& bodies, std::ofstream& csvFile, uint64_t timestamp)
{
    try
    {
        // Input validation
        if (bodies.empty())
        {
            return; // Nothing to write
        }

        if (!csvFile.is_open())
        {
            throw std::runtime_error("Failed to open CSV file - file not open");
        }

        // Lock the file for exclusive access
        std::lock_guard<std::mutex> lock(g_fileMutex);

        // Write CSV Header if file is empty (same as in single body function)
        if (csvFile.tellp() == 0)
        {
            std::stringstream headerStream;
            headerStream << "BodyID,Time";
            
            for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
            {
                const std::string& jointName = g_jointNames.at(static_cast<k4abt_joint_id_t>(joint));
                headerStream << "," << jointName << "_X"
                             << "," << jointName << "_Y"
                             << "," << jointName << "_Z"
                             << "," << jointName << "_CONFIDENCE";
            }
            headerStream << ",ANGLE" << std::endl;
            csvFile << headerStream.str();
        }

        // Create a buffer for all bodies
        std::stringstream batchStream;
        
        // Process all bodies
        for (const auto& body : bodies)
        {
            // Calculate arm angle
            Eigen::Vector3d jointPositionPelvis(
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_PELVIS)].position.xyz.x,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_PELVIS)].position.xyz.y,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_PELVIS)].position.xyz.z
            );

            Eigen::Vector3d jointPositionNeck(
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_NECK)].position.xyz.x,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_NECK)].position.xyz.y,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_NECK)].position.xyz.z
            );

            Eigen::Vector3d jointPositionNose(
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_NOSE)].position.xyz.x,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_NOSE)].position.xyz.y,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_NOSE)].position.xyz.z
            );

            Eigen::Vector3d jointPositionShoulder(
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_SHOULDER_RIGHT)].position.xyz.x,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_SHOULDER_RIGHT)].position.xyz.y,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_SHOULDER_RIGHT)].position.xyz.z
            );

            Eigen::Vector3d jointPositionElbow(
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_ELBOW_RIGHT)].position.xyz.x,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_ELBOW_RIGHT)].position.xyz.y,
                body.skeleton.joints[static_cast<int>(K4ABT_JOINT_ELBOW_RIGHT)].position.xyz.z
            );
			double angle = CalculateProjectedAngle(jointPositionPelvis, jointPositionNeck, jointPositionNose, jointPositionPelvis, jointPositionShoulder, jointPositionElbow);

            batchStream << body.id << "," << timestamp;
            
            for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
            {
                const k4a_float3_t& position = body.skeleton.joints[joint].position;
                batchStream << "," << position.xyz.x
                           << "," << position.xyz.y
                           << "," << position.xyz.z
                           << "," << body.skeleton.joints[joint].confidence_level;
            }
            batchStream << "," << angle << std::endl;
        }
        
        // Write all data at once
        csvFile << batchStream.str();
        
        // Flush to disk
        csvFile.flush();

        if (!csvFile.good())
        {
            throw std::runtime_error("Failed to write multiple bodies to CSV file - disk full or I/O error");
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error writing multiple bodies to CSV: " << e.what() << std::endl;
        throw;
    }
}

// Function to get the current timestamp in microseconds
uint64_t GetTimestamp()
{
#ifdef _WIN32
    // Windows
    LARGE_INTEGER frequency, counter;
    if (!QueryPerformanceFrequency(&frequency) || !QueryPerformanceCounter(&counter))
    {
        // Handle error if QueryPerformance functions fail
        std::cerr << "Error getting high-resolution performance counter" << std::endl;
        return 0;
    }
    return static_cast<uint64_t>(counter.QuadPart * 1000000.0 / frequency.QuadPart);
#else
    // Linux/Unix
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        std::cerr << "Error getting system time" << std::endl;
        return 0;
    }
    return static_cast<uint64_t>(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
#endif
}

void SaveColorImage(const k4a_image_t& colorImage, const std::string& folderPath, uint64_t timestamp, uint64_t frameCount)
{
	try
	{
		// Input validation
		if (colorImage == nullptr)
		{
			throw std::runtime_error("Invalid color image");
			}
		
		// If the folder does not exist, create it
        try {
            if (!std::filesystem::exists(folderPath))
            {
                if (!std::filesystem::create_directories(folderPath))
                {
                    throw std::runtime_error("Failed to create directory: " + folderPath);
                }
                std::cout << "Created directory: " << folderPath << std::endl;
            }
        }
        catch (const std::filesystem::filesystem_error& e) {
            throw std::runtime_error("Filesystem error: " + std::string(e.what()));
        }
		
		// Generate filename
		std::ostringstream ss;
		ss << folderPath << "/color_" << timestamp << "_" << std::setw(6) << std::setfill('0') << frameCount << ".jpg";
		std::string fileName = ss.str();
		
		// Get metadata of images
		uint8_t* buffer = k4a_image_get_buffer(colorImage);
		size_t bufferSize = k4a_image_get_size(colorImage);
		
		// Lock the file for exclusive access
		std::lock_guard<std::mutex> lock(g_fileMutex);
		
        // Save as jpeg
		std::ofstream outFile(fileName, std::ios::binary);
		if (!outFile)
		{
			throw std::runtime_error("Failed to open color image file: " + fileName);
		}
		
		// Write the color image data
		outFile.write(reinterpret_cast<const char*>(buffer), bufferSize);
		
		// Close the file
		outFile.close();
		if (!outFile.good())
		{
			throw std::runtime_error("Failed to write color image data");
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error saving color image: " << e.what() << std::endl;
		throw;
	}
}
