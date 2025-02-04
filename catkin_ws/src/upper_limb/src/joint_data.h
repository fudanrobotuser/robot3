#ifndef JOINT_DATA_H
#define JOINT_DATA_H

#include <queue>
#include <array>
#include <ros/ros.h>

#define QUEUE_MAX_SIZE 5000

class JointData {
private:
    std::queue<std::array<float, 9>> queue;
    const size_t max_size = QUEUE_MAX_SIZE;

public:
    bool enqueue(const std::array<float, 9>& arr);
    bool dequeue(std::array<float, 9>& arr);
    size_t getSize() const;
    void clear();
    float data[27] = {0.0f};
    int status = 0; // 0 默认 , 1 主栈启动中,  101, 主栈启动失败, 201 主栈启动完成, 
    //2 电机使能中 , 102 电机使能失败， 202 使能成功, 3 电机 回零中, 103 回零故障, 203 回零完成. 
    //4 执行位置运动 , 104 运动失败, 204 位置运动成功
    int status_old = 0;
};

#endif // JOINT_DATA_H
