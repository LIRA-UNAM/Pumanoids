#ifndef BUFFER_H
#define BUFFER_H

#include <vector>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

template<typename T>
class Buffer {
public:
    // Construct a buffer with a fixed maximum size.
    Buffer(size_t max_size) : max_size_(max_size) {}

    // Add data using the current timestamp.
    void add(const T& data) {
        add(data, rclcpp::Clock().now());
    }

    // Add data using the supplied timestamp.
    void add(const T& data, const rclcpp::Time& timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Remove the oldest sample when the buffer is full.
        if(data_.size() >= max_size_) {
            data_.erase(data_.begin());
            timestamps_.erase(timestamps_.begin());
        }
        
        data_.push_back(data);
        timestamps_.push_back(timestamp);
    }

    // Return the sample at the specified index.
    T get(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(index >= data_.size()) {
            throw std::out_of_range("Index out of range");
        }
        return data_[index];
    }
    
    // Return the sample and timestamp nearest to the requested time.
    bool get_nearest(const rclcpp::Time& target_time, T& data_out, rclcpp::Time& time_out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(data_.empty()) {
            return false;
        }

        size_t nearest_idx = 0;
        auto min_diff = std::abs((timestamps_[0] - target_time).nanoseconds());

        // Find the timestamp with the smallest absolute difference.
        for(size_t i = 1; i < timestamps_.size(); ++i) {
            auto diff = std::abs((timestamps_[i] - target_time).nanoseconds());
            if(diff < min_diff) {
                min_diff = diff;
                nearest_idx = i;
            }
        }

        data_out = data_[nearest_idx];
        time_out = timestamps_[nearest_idx];
        return true;
    }

    // Return the timestamp at the specified index.
    rclcpp::Time get_timestamp(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if(index >= timestamps_.size()) {
            throw std::out_of_range("Index out of range");
        }
        return timestamps_[index];
    }

    // Return the current number of buffered samples.
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.size();
    }

    // Clear the buffer.
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.clear();
        timestamps_.clear();
    }

private:
    std::vector<T> data_;                    // Stored samples
    std::vector<rclcpp::Time> timestamps_;   // Corresponding timestamps
    const size_t max_size_;                  // Maximum sample count
    mutable std::mutex mutex_;               // Thread-safety lock
};

#endif // BUFFER_H
