#include "hikcamera/shm.hpp"

namespace hikcamera {

auto SHMInit(const std::string& shm_path_name, size_t shm_size) -> std::expected<int, std::string> {
    int shm_fd = shm_open(shm_path_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        return std::unexpected("Failed to create shared memory object");
    }
    if (ftruncate(shm_fd, shm_size) == -1) {
        return std::unexpected("Failed to set size of shared memory object");
    }
    auto image_shm = reinterpret_cast<imageSHM*>(
        mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&image_shm->mutex, &mutex_attr);
    sem_init(&image_shm->sem, 1, 0);
    image_shm->read_index         = -1;
    image_shm->write_index        = -1;
    image_shm->counter            = 1;
    image_shm->is_shm_initialized.store(true, std::memory_order_release);
    image_shm->frame_counter.store(0, std::memory_order_release);
    return { shm_fd };
}

auto SHMWrite(int shm_fd, const Camera::Image& data)
    -> std::expected<void, std::string> {
    auto image_shm = reinterpret_cast<imageSHM*>(mmap(nullptr,
        sizeof(imageSHM), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (image_shm == MAP_FAILED) {
        return std::unexpected("Failed to map shared memory object");
    }
    pthread_mutex_lock(&image_shm->mutex);
    image_shm->write_index++;
    auto write_index = image_shm->write_index % SLOT_NUM;
    std::memcpy(
        image_shm->imagedata[write_index], data.mat.data, data.mat.total() * data.mat.elemSize());
    image_shm->timestamp[write_index] = data.timestamp;
    pthread_mutex_unlock(&image_shm->mutex);
    image_shm->frame_counter.fetch_add(1, std::memory_order_release);
    sem_post(&image_shm->sem);
    munmap(image_shm, sizeof(imageSHM));
    return { };
}

auto SHMRead(int shm_fd, cv::Mat& out_mat, std::chrono::steady_clock::time_point& out_ts, int width,
    int height) -> std::expected<void, std::string> {
    auto image_shm = reinterpret_cast<imageSHM*>(mmap(nullptr,
        sizeof(imageSHM), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (image_shm == MAP_FAILED) {
        return std::unexpected("Failed to map shared memory object");
    }
    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += 1;
    sem_timedwait(&image_shm->sem, &timeout);
    pthread_mutex_lock(&image_shm->mutex);
    if (image_shm->is_shm_initialized.load(std::memory_order_acquire) == 1) {
        if (image_shm->read_index >= image_shm->write_index) {
            image_shm->read_index = image_shm->write_index;
        } else {
            image_shm->read_index++;
        }
        auto read_index = (image_shm->read_index) % SLOT_NUM;
        auto& frame     = image_shm->imagedata[read_index];
        out_mat         = cv::Mat(height, width, CV_8UC3, frame).clone();
        out_ts          = image_shm->timestamp[read_index];
    } else {
        pthread_mutex_unlock(&image_shm->mutex);
        munmap(image_shm, sizeof(imageSHM));
        return std::unexpected("Shared memory is not initialized");
    }
    pthread_mutex_unlock(&image_shm->mutex);
    munmap(image_shm, sizeof(imageSHM));
    return { };
}

auto SHMClose(int shm_fd) -> std::expected<bool, std::string> {
    if (close(shm_fd) == -1) {
        return std::unexpected("Failed to close shared memory object");
    }
    return { true };
}

auto SHMUnlink(const std::string& shm_path_name) -> std::expected<bool, std::string> {
    if (shm_unlink(shm_path_name.c_str()) == -1) {
        return std::unexpected("Failed to unlink shared memory object");
    }
    return { true };
}

} // namespace hikcamera
