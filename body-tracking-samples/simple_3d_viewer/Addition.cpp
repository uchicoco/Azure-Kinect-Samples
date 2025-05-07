#include <fstream>
// No changes needed in this file for now.
#include <iostream>
#include <mutex>
#include<opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <k4a/k4a.h>
#include <k4abt.h>

#include <BodyTrackingHelpers.h>
#include <Eigen/Dense>
#include <Windows.h>

#include "Addition.h"
#include "AngleCalculator.h"
#include "Calibration.h"

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

// Funciton to transform body data into std::vector<cv::Point3f>
std::vector<cv::Point3f> ConvertBodyToPoints(const k4abt_body_t& body) {
    std::vector<cv::Point3f> points;

    // Add jont points
    for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); ++joint) {
        const k4a_float3_t& position = body.skeleton.joints[joint].position;
        points.emplace_back(position.xyz.x, position.xyz.y, position.xyz.z);
    }

    return points;
}

std::vector<Transform>CalibrateCameras(const k4abt_body_t& body1, const k4abt_body_t& body2, const k4abt_body_t& body3, const k4abt_body_t& body_m)
{
	// Transform body data into point vectors
    std::vector<cv::Point3f> subPoints1 = ConvertBodyToPoints(body1);
    std::vector<cv::Point3f> subPoints2 = ConvertBodyToPoints(body2);
    std::vector<cv::Point3f> subPoints3 = ConvertBodyToPoints(body3);
    std::vector<cv::Point3f> mainPoints = ConvertBodyToPoints(body_m);

	// Calibrate the cameras
    CalibrationResult result = calibration(subPoints1, subPoints2, subPoints3, mainPoints);

    if (!result.success) {
        throw std::runtime_error("Calibration failed!");
    }
    return result.transforms;
}

k4abt_body_t ApplyTransformToBody(const k4abt_body_t& body, const Transform& transform)
{
    try
    {
        // Create a copy of the input body that we'll modify
        k4abt_body_t transformedBody = body;
        
        // Apply the transformation to each joint position
        for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); ++joint)
        {
            // Extract the current position
            const k4a_float3_t& originalPosition = body.skeleton.joints[joint].position;
            
            // Convert to cv::Point3f for the transform.apply() method
            cv::Point3f point(originalPosition.xyz.x, originalPosition.xyz.y, originalPosition.xyz.z);
            
            // Apply the transformation
            cv::Point3f transformedPoint = transform.apply(point);
            
            // Store the transformed position back
            transformedBody.skeleton.joints[joint].position.xyz.x = transformedPoint.x;
            transformedBody.skeleton.joints[joint].position.xyz.y = transformedPoint.y;
            transformedBody.skeleton.joints[joint].position.xyz.z = transformedPoint.z;
            
            // Note: We retain the original orientation and confidence level
        }
        
        return transformedBody;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error applying transformation to body: " << e.what() << std::endl;
        throw;
    }
}

std::vector<k4abt_body_t> ApplyTransformToBodies(const std::vector<k4abt_body_t>& bodies, const Transform& transform)
{
    try
    {
        // Input validation
        if (bodies.empty())
        {
            return std::vector<k4abt_body_t>(); // Return empty vector
        }
        
        std::vector<k4abt_body_t> transformedBodies;
        transformedBodies.reserve(bodies.size());
        
        // Transform each body
        for (const auto& body : bodies)
        {
            // Create a copy of the input body that we'll modify
            k4abt_body_t transformedBody = body;
            
            // Apply the transformation to each joint position
            for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); ++joint)
            {
                // Extract the current position
                const k4a_float3_t& originalPosition = body.skeleton.joints[joint].position;
                
                // Convert to cv::Point3f for the transform.apply() method
                cv::Point3f point(originalPosition.xyz.x, originalPosition.xyz.y, originalPosition.xyz.z);
                
                // Apply the transformation
                cv::Point3f transformedPoint = transform.apply(point);
                
                // Store the transformed position back
                transformedBody.skeleton.joints[joint].position.xyz.x = transformedPoint.x;
                transformedBody.skeleton.joints[joint].position.xyz.y = transformedPoint.y;
                transformedBody.skeleton.joints[joint].position.xyz.z = transformedPoint.z;
            }
            
            transformedBodies.push_back(transformedBody);
        }
        
        return transformedBodies;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error applying transformation to bodies: " << e.what() << std::endl;
        throw;
    }
}

bool IsSameBody(const k4abt_body_t& body1, const k4abt_body_t& body2, float distanceThresholdMm = 100.0f)
{
    try
    {
        // Important joints to compare
        static const k4abt_joint_id_t keyJoints[] = {
            K4ABT_JOINT_PELVIS,
            K4ABT_JOINT_SPINE_CHEST,
            K4ABT_JOINT_NECK,
            K4ABT_JOINT_HEAD,
            K4ABT_JOINT_SHOULDER_LEFT,
            K4ABT_JOINT_SHOULDER_RIGHT,
            K4ABT_JOINT_HIP_LEFT,
            K4ABT_JOINT_HIP_RIGHT
        };
        static const int keyJointCount = sizeof(keyJoints) / sizeof(keyJoints[0]);

        float totalDistance = 0.0f;

        // Compare the distance of each important joint
        for (int i = 0; i < keyJointCount; ++i)
        {
            const k4abt_joint_id_t jointId = keyJoints[i];

            // Get positions regardless of confidence level
            const k4a_float3_t& pos1 = body1.skeleton.joints[jointId].position;
            const k4a_float3_t& pos2 = body2.skeleton.joints[jointId].position;

            // Calculate Euclidean distance
            float dx = pos1.xyz.x - pos2.xyz.x;
            float dy = pos1.xyz.y - pos2.xyz.y;
            float dz = pos1.xyz.z - pos2.xyz.z;
            float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            totalDistance += distance;
        }

        // Calculate average distance
        float averageDistance = totalDistance / keyJointCount;

        // Return true if average distance is less than threshold
        return averageDistance <= distanceThresholdMm;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error in IsSameBody: " << e.what() << std::endl;
        return false;
    }
}

std::vector<k4abt_body_t> MergeAndTransformBodies(
    const std::vector<k4abt_body_t>& bodies1,
    const std::vector<k4abt_body_t>& bodies2,
    const std::vector<k4abt_body_t>& bodies3,
    const std::vector<k4abt_body_t>& bodies4,
    const Transform& transform1,
    const Transform& transform2,
    const Transform& transform3)
{
    // Step 1: Apply transformations to align coordinates
    std::vector<k4abt_body_t> transformedBodies1 = ApplyTransformToBodies(bodies1, transform1);
    std::vector<k4abt_body_t> transformedBodies2 = ApplyTransformToBodies(bodies2, transform2);
    std::vector<k4abt_body_t> transformedBodies3 = ApplyTransformToBodies(bodies3, transform3);

    // Step 2: Combine all bodies into a single vector
    std::vector<k4abt_body_t> allBodies = transformedBodies1;
    allBodies.insert(allBodies.end(), transformedBodies2.begin(), transformedBodies2.end());
    allBodies.insert(allBodies.end(), transformedBodies3.begin(), transformedBodies3.end());
    allBodies.insert(allBodies.end(), bodies4.begin(), bodies4.end());

    // Step 3: Merge bodies based on IsSameBody
    std::vector<k4abt_body_t> mergedBodies;
    std::vector<bool> used(allBodies.size(), false);

    for (size_t i = 0; i < allBodies.size(); ++i)
    {
        if (used[i]) continue;

        k4abt_body_t mergedBody = allBodies[i];
        float totalConfidence[K4ABT_JOINT_COUNT] = { 0 };
        float weightedX[K4ABT_JOINT_COUNT] = { 0 };
        float weightedY[K4ABT_JOINT_COUNT] = { 0 };
        float weightedZ[K4ABT_JOINT_COUNT] = { 0 };

        for (int joint = 0; joint < K4ABT_JOINT_COUNT; ++joint)
        {
            const k4a_float3_t& pos = mergedBody.skeleton.joints[joint].position;
            weightedX[joint] = pos.xyz.x * mergedBody.skeleton.joints[joint].confidence_level;
            weightedY[joint] = pos.xyz.y * mergedBody.skeleton.joints[joint].confidence_level;
            weightedZ[joint] = pos.xyz.z * mergedBody.skeleton.joints[joint].confidence_level;
            totalConfidence[joint] = mergedBody.skeleton.joints[joint].confidence_level;
        }

        for (size_t j = i + 1; j < allBodies.size(); ++j)
        {
            if (used[j]) continue;

            if (IsSameBody(mergedBody, allBodies[j]))
            {
                used[j] = true;

                for (int joint = 0; joint < K4ABT_JOINT_COUNT; ++joint)
                {
                    const k4a_float3_t& pos = allBodies[j].skeleton.joints[joint].position;
                    float confidence = allBodies[j].skeleton.joints[joint].confidence_level;

                    weightedX[joint] += pos.xyz.x * confidence;
                    weightedY[joint] += pos.xyz.y * confidence;
                    weightedZ[joint] += pos.xyz.z * confidence;
                    totalConfidence[joint] += confidence;
                }
            }
        }

        // Finalize joint positions with confidence-weighted averaging
        for (int joint = 0; joint < K4ABT_JOINT_COUNT; ++joint)
        {
            if (totalConfidence[joint] > 0)
            {
                mergedBody.skeleton.joints[joint].position.xyz.x = weightedX[joint] / totalConfidence[joint];
                mergedBody.skeleton.joints[joint].position.xyz.y = weightedY[joint] / totalConfidence[joint];
                mergedBody.skeleton.joints[joint].position.xyz.z = weightedZ[joint] / totalConfidence[joint];
            }
        }

        mergedBodies.push_back(mergedBody);
    }

    // Step 4: Assign unique IDs to merged bodies
    static uint32_t nextBodyId = 0;
    for (auto& body : mergedBodies)
    {
        body.id = nextBodyId++;
    }

    return mergedBodies;
}

// ...existing code...