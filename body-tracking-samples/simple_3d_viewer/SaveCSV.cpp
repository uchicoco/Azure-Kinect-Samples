#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>

#include <BodyTrackingHelpers.h>

#include "SaveCSV.h"

void SaveJointPositionsToCSV(const k4abt_body_t& body, std::ofstream& csvFile, uint64_t frameCount)
{
    try
    {
        if (!csvFile.is_open())
        {
            throw std::runtime_error("Failed to open CSV file");
        }

        // Write CSV Header if file is empty
        if (csvFile.tellp() == 0)
        {
            std::stringstream headerStream;
            headerStream << "BodyID,FrameCount";
            
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
        dataStream << body.id << "," << frameCount;
        
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
    }
}