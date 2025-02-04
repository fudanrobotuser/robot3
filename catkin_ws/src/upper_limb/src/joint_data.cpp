#include "joint_data.h"

bool JointData::enqueue(const std::array<float, 9>& arr) {
    if (queue.size() >= max_size) {
        ROS_ERROR("Queue is full, cannot enqueue new data!");
        return false;
    }
    queue.push(arr);
    return true;
}

bool JointData::dequeue(std::array<float, 9>& arr) {
    if (queue.empty()) {
        ROS_WARN("Queue is empty, cannot dequeue!");
        return false;
    }
    arr = queue.front();
    queue.pop();
    return true;
}

size_t JointData::getSize() const {
    return queue.size();
}

void JointData::clear() {
    while (!queue.empty()) {
        queue.pop();
    }
}
