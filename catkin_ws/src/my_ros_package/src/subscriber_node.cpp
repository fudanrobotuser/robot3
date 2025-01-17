#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // 检查数据长度是否为 36
    if (msg->data.size() != 36) {
        ROS_WARN("Received array length is not 36. Actual length: %zu", msg->data.size());
        return;
    }

    // 定义数组的行数和列数
    const int rows = 12;
    const int cols = 3;

    // 打印数组
    ROS_INFO("Received a %dx%d array:", rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // 计算当前元素在一维数组中的索引
            int index = i * cols + j;
            printf("%8.6f ", msg->data[index]); // 格式化输出
        }
        printf("\n"); // 换行
    }
    printf("\n"); // 打印空行分隔每次输出
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "array_reshape_printer");
    ros::NodeHandle nh;

    // 创建一个订阅者，监听 /motor_feedback topic
    ros::Subscriber sub = nh.subscribe("/motor_feedback", 10, arrayCallback);

    // 进入循环
    ros::spin();

    return 0;
}