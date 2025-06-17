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

// Mutex for file access synchronization
static std::mutex g_fileMutex;
static std::mutex g_pipeMutex;

// Named pipe file descriptor
static int g_pipefd = -1;
static const char* PIPE_NAME = "/tmp/angle_data_pipe";

bool InitializeNamedPipe()
{
    // Remove existing pipe if it exists
    unlink(PIPE_NAME);
    
    // Create named pipe
    if (mkfifo(PIPE_NAME, 0666) == -1) {
        std::cerr << "Failed to create named pipe: " << PIPE_NAME << std::endl;
        return false;
    }
    
    std::cout << "Named pipe created: " << PIPE_NAME << std::endl;
    return true;
}

bool OpenNamedPipe()
{
    // Open pipe for writing (non-blocking)
    g_pipefd = open(PIPE_NAME, O_WRONLY | O_NONBLOCK);
    if (g_pipefd == -1) {
        // If no reader is available, this is normal
        return false;
    }
    return true;
}

void SendAngleDataToPipe(uint32_t body_id, double right_arm_angle, double left_arm_angle, double legs_angle, uint64_t timestamp)
{
    if (g_pipefd == -1) {
        // Try to open pipe if not already open
        if (!OpenNamedPipe()) {
            return; // No reader available
        }
    }
    
    std::lock_guard<std::mutex> lock(g_pipeMutex);
    
    // Format: timestamp,body_id,right_angle,left_angle,legs_angle\n
    std::stringstream ss;
    ss << timestamp << "," << body_id << "," 
       << right_arm_angle << "," << left_arm_angle << "," << legs_angle << "\n";
    
    std::string data = ss.str();
    ssize_t written = write(g_pipefd, data.c_str(), data.length());
    
    if (written == -1) {
        // Pipe is broken, close and reset
        close(g_pipefd);
        g_pipefd = -1;
    }
}

void CleanupNamedPipe()
{
    if (g_pipefd != -1) {
        close(g_pipefd);
        g_pipefd = -1;
    }
    unlink(PIPE_NAME);
}