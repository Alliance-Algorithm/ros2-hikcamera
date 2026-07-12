#pragma once

#include <atomic>
#include <chrono>
#include <expected>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/core.hpp>

#include "hikcamera/capturer.hpp"

#define SLOT_NUM 4
#define MAX_IMAGE_SIZE 36864000

namespace hikcamera {

typedef struct imageSHM {
    sem_t sem;
    pthread_mutex_t mutex;
    std::atomic<bool> is_shm_initialized;
    unsigned int read_index  = -1;
    unsigned int write_index = -1;
    unsigned int counter     = 0;
    std::chrono::steady_clock::time_point timestamp[SLOT_NUM];
    char imagedata[SLOT_NUM][MAX_IMAGE_SIZE];
    std::atomic<uint64_t> frame_counter{0};
} imageSHM;

auto SHMInit(const std::string& shm_path_name, size_t shm_size) -> std::expected<int, std::string>;
auto SHMWrite(int shm_fd, const Camera::Image& data) -> std::expected<void, std::string>;
auto SHMRead(int shm_fd, cv::Mat& out_mat, std::chrono::steady_clock::time_point& out_ts,
    int width, int height) -> std::expected<void, std::string>;
auto SHMClose(int shm_fd) -> std::expected<bool, std::string>;
auto SHMUnlink(const std::string& shm_path_name) -> std::expected<bool, std::string>;

} // namespace hikcamera
