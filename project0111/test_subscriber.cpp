#include <pthread.h>
#include <iostream>
#include <vector>
#include "rt_ethercat.hpp"
#include <string>
#include <thread>
#include <iomanip>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "ecat_data_buffer.h" // 包含共享内存操作的头文件

using namespace std;
using namespace ETHERCAT_SPACE;

// 定义共享内存键值和大小
#define SHM_KEY 123456
#define SHM_SIZE (1024 * 1024 * 2)

// 定义电机数量
#define LEFT_MOTOR_NUM 9
#define RIGHT_MOTOR_NUM 6
#define TOTAL_MOTOR_NUM (LEFT_MOTOR_NUM + RIGHT_MOTOR_NUM)



// 定义共享内存指针
static ECAT_BUF_SHM_STRU* ecat_shm_pointer = nullptr;

// 定义电机对象
Ecat_motor motors_left("enp2s0", 1000, LEFT_MOTOR_NUM);
Ecat_motor motors_right("enp3s0", 1000, RIGHT_MOTOR_NUM);

// 定义反馈和参考数据结构
GROUP_FEEDBACK feedback;
GROUP_REFERENCE reference;

// 定义控制指令和电机数据
vector<ImpedanceModeCmd> Icmomond_left;
vector<SpeedModeCmd> Scmomond_left;
vector<PositionModeCmd> Pcmomond_left;
vector<motor_data> Data_left;

vector<ImpedanceModeCmd> Icmomond_right;
vector<SpeedModeCmd> Scmomond_right;
vector<PositionModeCmd> Pcmomond_right;
vector<motor_data> Data_right;


// 初始化共享内存
bool init_shared_memory() {
    int shmid = shmget(SHM_KEY, SHM_SIZE, 0666);
    if (shmid == -1) {
        perror("shmget");
        return false;
    }

    void* appPtr = shmat(shmid, 0, 0);
    if (appPtr == (void*)-1) {
        perror("shmat");
        return false;
    }

    if (!edb_init(appPtr, SHM_SIZE, false)) {
        cerr << "Failed to initialize shared memory!" << endl;
        return false;
    }

    ecat_shm_pointer = (ECAT_BUF_SHM_STRU*)appPtr;
    memset(&feedback, 0, sizeof(feedback));
    memset(&reference, 0, sizeof(reference));
    return true;
}

// 读取电机反馈数据并写入共享内存
void read_and_push_feedback() {
    // 读取左侧电机数据
    for (int i = 0; i < LEFT_MOTOR_NUM; i++) {
        motors_left.Id_data(i + 1, &Data_left[i]); // 读取电机数据到 Data_left
        feedback.motor_fdbk[i].feedbk_postion = Data_left[i].angle;    // 位置
        feedback.motor_fdbk[i].feedbk_speed = Data_left[i].velocity;   // 速度
        feedback.motor_fdbk[i].feedbk_torque = Data_left[i].torque;    // 扭力
    }

    // 读取右侧电机数据
    for (int i = 0; i < RIGHT_MOTOR_NUM; i++) {
        motors_right.Id_data(i + 1, &Data_right[i]); // 读取电机数据到 Data_right
        feedback.motor_fdbk[LEFT_MOTOR_NUM + i].feedbk_postion = Data_right[i].angle;    // 位置
        feedback.motor_fdbk[LEFT_MOTOR_NUM + i].feedbk_speed = Data_right[i].velocity;   // 速度
        feedback.motor_fdbk[LEFT_MOTOR_NUM + i].feedbk_torque = Data_right[i].torque;    // 扭力
    }

    // 将反馈数据推送到共享内存
    if (!edb_push_fdbk(&feedback)) {
        cerr << "Failed to push feedback to shared memory!" << endl;
    }
}

// 读取共享内存中的参考数据并写入电机控制指令
void read_and_apply_reference() {
    if (!edb_pull_ref(&reference)) {
        cerr << "Failed to pull reference from shared memory!" << endl;
        return;
    }

    // 应用左侧电机参考数据
    for (int i = 0; i < LEFT_MOTOR_NUM; i++) {
        // 将 reference 中的数据赋值给 Pcmomond_left
        Pcmomond_left[i].angle = reference.motor_ref[i].target_postion; // 目标位置
        Pcmomond_left[i].Vkp = 3;                                      // 位置环比例增益
        Pcmomond_left[i].Vki = 700;                                    // 位置环积分增益
        Pcmomond_left[i].KP = 1;                                       // 刚度
        Pcmomond_left[i].KD = 0;                                       // 阻尼

        // 发送控制指令
        motors_left.Id_command(i + 1, &Pcmomond_left[i]);
    }

    // 应用右侧电机参考数据
    for (int i = 0; i < RIGHT_MOTOR_NUM; i++) {
        // 将 reference 中的数据赋值给 Pcmomond_right
        Pcmomond_right[i].angle = reference.motor_ref[LEFT_MOTOR_NUM + i].target_postion; // 目标位置
        Pcmomond_right[i].Vkp = 3;                                                       // 位置环比例增益
        Pcmomond_right[i].Vki = 700;                                                     // 位置环积分增益
        Pcmomond_right[i].KP = 1;                                                        // 刚度
        Pcmomond_right[i].KD = 0;                                                        // 阻尼

        // 发送控制指令
        motors_right.Id_command(i + 1, &Pcmomond_right[i]);
    }
}

// 处理用户输入
void handle_user_input(char val) {
    int cnt = 1;
    int Ecat_id = 1;
    float set_value = 0;
    float read_value = 0;

    switch (val) {
        case 's':
            std::cout << "input number\n";
            std::cin >> cnt;
            if (cnt < 1 || cnt > motors_left._motor_num) {
                cnt = 1;
                std::cout << "error input,now cnt is 1\n";
            }
            break;
        case 'a':
            std::cout << "input Ecat_id\n";
            std::cin >> Ecat_id;
            if (Ecat_id < 1 || Ecat_id > motors_left._motor_num) {
                Ecat_id = 1;
                std::cout << "error input,now Ecat_id is 1\n";
            }
            break;
        case 'd': // 小写 d，失能左侧电机群中的某个电机
            motors_left.Id_control_order(Ecat_id, DISABLE_MODE);
            std::cout << "左侧电机 " << Ecat_id << " 已失能\n";
            break;
        case 'D': // 大写 D，失能所有电机
            for (int i = 1; i <= motors_left._motor_num; i++) {
                motors_left.Id_control_order(i, DISABLE_MODE);
            }
            for (int i = 1; i <= motors_right._motor_num; i++) {
                motors_right.Id_control_order(i, DISABLE_MODE);
            }
            std::cout << "所有电机已失能\n";
            break;
        case 'e': // 小写 e，使能左侧电机群中的某个电机
            motors_left.Id_control_order(Ecat_id, ENABLE_MODE);
            std::cout << "左侧电机 " << Ecat_id << " 已使能\n";
            break;
        case 'E': // 大写 E，使能所有电机
            for (int i = 1; i <= motors_left._motor_num; i++) {
                motors_left.Id_control_order(i, ENABLE_MODE);
            }
            for (int i = 1; i <= motors_right._motor_num; i++) {
                motors_right.Id_control_order(i, ENABLE_MODE);
            }
            std::cout << "所有电机已使能\n";
            break;
        case 'z': // 调零
            for (int cnt = 1; cnt <= motors_left._motor_num; cnt++)
                motors_left.control_order(cnt, ZERO_MODE); // 使用 Id_command
            for (int cnt = 1; cnt <= motors_right._motor_num; cnt++)
                motors_right.control_order(cnt, ZERO_MODE); // 使用 Id_command
            printf("所有电机角度调0\n\r");
            break;
        case 'p': // 小写 p，位置环控制（仅左侧电机）
            Pcmomond_left[cnt - 1].angle = 0;                    // 目标位置
            Pcmomond_left[cnt - 1].Vkp = 3;                      // 位置环比例增益
            Pcmomond_left[cnt - 1].Vki = 700;                    // 位置环积分增益
            Pcmomond_left[cnt - 1].KP = 1;                       // 刚度
            Pcmomond_left[cnt - 1].KD = 0;                       // 阻尼
            motors_left.Id_command(Ecat_id, &Pcmomond_left[cnt - 1]); // 发送控制指令
            std::cout << "左侧电机 " << Ecat_id << " 位置环控制已设置\n";
            break;
        case 'P': // 大写 P，所有电机运行到位置 0
            for (int i = 1; i <= motors_left._motor_num; i++) {
                Pcmomond_left[i - 1].angle = 0;
                Pcmomond_left[i - 1].Vkp = 3;
                Pcmomond_left[i - 1].Vki = 700;
                Pcmomond_left[i - 1].KP = 1;
                Pcmomond_left[i - 1].KD = 0;
                motors_left.Id_command(i, &Pcmomond_left[i - 1]); // 使用 Id_command
            }
            for (int i = 1; i <= motors_right._motor_num; i++) {
                Pcmomond_right[i - 1].angle = 0;
                Pcmomond_right[i - 1].Vkp = 3;
                Pcmomond_right[i - 1].Vki = 700;
                Pcmomond_right[i - 1].KP = 1;
                Pcmomond_right[i - 1].KD = 0;
                motors_right.Id_command(i, &Pcmomond_right[i - 1]); // 使用 Id_command
            }
            std::cout << "所有电机正在运行到位置 0\n";
            break;
        case 'K': // 用户输入 K
            read_and_push_feedback(); // 读取电机反馈并写入共享内存
            read_and_apply_reference(); // 读取共享内存中的参考数据并写入电机控制指令
            cout << "Feedback pushed and reference applied!" << endl;
            break;
        default:
            break;
    }
}

pthread_t thread1;
//EtherCat线程读写实例
void Ethercat_syncThread(){
  while(1){
      motors_right.rt_ethercat_run();  //程序内部包含与从站线程周期同步，此函数处理时间约300Us左右
      motors_left.rt_ethercat_run();
  }
}

int main(int argc, char **argv) {
    // 初始化控制指令和电机数据
    Icmomond_left.resize(motors_left._motor_num);
    Scmomond_left.resize(motors_left._motor_num);
    Pcmomond_left.resize(motors_left._motor_num);
    Data_left.resize(motors_left._motor_num);

    Icmomond_right.resize(motors_right._motor_num);
    Scmomond_right.resize(motors_right._motor_num);
    Pcmomond_right.resize(motors_right._motor_num);
    Data_right.resize(motors_right._motor_num);


    // 初始化共享内存
    if (!init_shared_memory()) {
        cerr << "Shared memory initialization failed!" << endl;
        return EXIT_FAILURE;
    }

    // 开启电机通信线程
    std::thread rcv_thread1 = std::thread(&Ethercat_syncThread);
    usleep(100000); // 延时 100ms

    // 主循环
    char val;
    while (true) {
        std::cin >> val;
        handle_user_input(val); // 处理用户输入
    }

    return 0;
}