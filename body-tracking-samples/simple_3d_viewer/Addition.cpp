#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <mutex>
#include <vector>

#include <BodyTrackingHelpers.h>
#include <Windows.h>

#include "Addition.h"

// Mutex for file access synchronization
static std::mutex g_fileMutex;

void SaveJointPositionsToCSV(const k4abt_body_t& body, std::ofstream& csvFile, uint64_t timestamp)
{
    try
    {
        // Input validation
        if (!csvFile.is_open())
        {
            throw std::runtime_error("Failed to open CSV file - file not open");
        }

        // Lock the file for exclusive access
        std::lock_guard<std::mutex> lock(g_fileMutex);

        // Write CSV Header if file is empty
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
            headerStream << std::endl;
            csvFile << headerStream.str();
        }

        // Build data row in a string stream for better performance
        std::stringstream dataStream;
        dataStream << body.id << "," << timestamp;
        
        // Write joint positions and confidence levels
        for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
        {
            const k4a_float3_t& position = body.skeleton.joints[joint].position;
            dataStream << "," << position.xyz.x
                       << "," << position.xyz.y
                       << "," << position.xyz.z
                       << "," << body.skeleton.joints[joint].confidence_level;
        }
        dataStream << std::endl;
        
        // Write the complete row at once
        csvFile << dataStream.str();

        if (!csvFile.good())
        {
            throw std::runtime_error("Failed to write to CSV file - disk full or I/O error");
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error writing CSV: " << e.what() << std::endl;
        // Re-throw to allow caller to handle the error
        throw;
    }
}

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
            headerStream << std::endl;
            csvFile << headerStream.str();
        }

        // Create a buffer for all bodies
        std::stringstream batchStream;
        
        // Process all bodies
        for (const auto& body : bodies)
        {
            batchStream << body.id << "," << timestamp;
            
            for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
            {
                const k4a_float3_t& position = body.skeleton.joints[joint].position;
                batchStream << "," << position.xyz.x
                           << "," << position.xyz.y
                           << "," << position.xyz.z
                           << "," << body.skeleton.joints[joint].confidence_level;
            }
            batchStream << std::endl;
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

