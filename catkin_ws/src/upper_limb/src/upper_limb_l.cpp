#define RIGHT_ARM 
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <thread>
#include "joint_data.h"
#include "ethercat_interface_l.h"

ros::Publisher motor_feedback_pub;
ros::Publisher motor_status_pub;
ros::Subscriber motor_action_sub;
ros::Subscriber motor_command_sub;
ros::Timer motor_feedback_timer;

int action_value = 0;

void motor_action_callback(const std_msgs::Int32::ConstPtr& msg) {
    action_value = msg->data;
    
    if (action_value == 1) {
        running_thread = false;
    }
    else if (action_value == 2 ) {
        action_value_ = action_value;
    }    
    else if (action_value == 3 ) {
        action_value_ = action_value;
    }     
    else if (action_value == 5 ) {
        action_value_ = action_value;
    }      
    else if(action_value == 10){
        consoleJoints();
    }
    else if(action_value == 11){
        consoleQueue();
    }    

    ROS_INFO("Received action value: %d", action_value);
}

void motor_command_callback(const std_msgs::Float32MultiArray::ConstPtr& cmd_msg) {
    if (cmd_msg->data.size() != 7) {
        ROS_ERROR("Received invalid motor command, expected length: 7, received: %lu", cmd_msg->data.size());
        return;
    }

    std::array<float, 9> motor_data;
    for (size_t i = 0; i < 7; ++i) {
        motor_data[i] = cmd_msg->data[i];
    }

    leftArm.enqueue(motor_data);
    if(leftArm.getSize()>100){
        action_value_ = 4;
    }
}

void motor_feedback_timer_callback(const ros::TimerEvent&) {
    std_msgs::Float32MultiArray array_msg;
    array_msg.data.resize(27);
    for(int i=0;i<27;i++){
        array_msg.data[i] = leftArm.data[i];
    }
    motor_feedback_pub.publish(array_msg);
    if(leftArm.status != leftArm.status_old){
        std_msgs::Int32 msg_status ;
        msg_status.data = leftArm.status;
        motor_status_pub.publish(msg_status);
        leftArm.status_old = leftArm.status;
    }
}

int main(int argc, char** argv) {
    Igh_init();
    Igh_master_activate();

    ros::init(argc, argv, "upper_limb_l");
    ros::NodeHandle nh;
    motor_feedback_pub = nh.advertise<std_msgs::Float32MultiArray>("motor_feedback_l", 1);
    motor_status_pub = nh.advertise<std_msgs::Int32>("motor_status_l", 1);
    motor_action_sub = nh.subscribe("/motor_action_l", 1, motor_action_callback);
    motor_command_sub = nh.subscribe("/motor_command_l", 1, motor_command_callback);
    motor_feedback_timer = nh.createTimer(ros::Duration(1.0 / 250.0), motor_feedback_timer_callback);
    std::thread rcv_thread1(Ethercat_syncThread);
    ros::spin();
    ros::shutdown();

    // rcv_thread1.join();
    return 0;
}
