/**
 * 控制左臂.7个电机
*/
#include <stdbool.h>
#include "ecrt.h"
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/types.h>
#include <malloc.h>
#include <inttypes.h>
#include <math.h>  // 包含数学库

#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#define THREAD_PRIORITY 60  // Priority for the real-time thread
#define PI 3.1415926

static unsigned int counter = 0;
static unsigned int sync_ref_counter = 0;
static unsigned int sync_ref_counter2 = 0;
static struct timespec apptime;

// 同步周期时钟配置,8毫秒下发周期控制
#define TASK_FREQUENCY 125 /* Hz */
#define CLOCK_TO_USE CLOCK_REALTIME
#define TIMEOUT_CLEAR_ERROR (1 * TASK_FREQUENCY) /* clearing error timeout */
// 时间控制相关函数
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / TASK_FREQUENCY)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
const struct timespec cycletime = {0, PERIOD_NS};

#define E_START2 0
#define E_STOP2 6

// EtherCAT
static ec_master_t* master2 = NULL;
static ec_master_state_t master_state2 = {};
static ec_domain_t* domain2 = NULL;
static ec_domain_state_t domain2_state = {};

int ii = 0;
bool toZero = true;
int toZeroOffset = 200;

/****************************************************************************/

// process data
static uint8_t* domain1_pd = NULL;
static uint8_t* domain2_pd = NULL;

#define VidPid 0x00522227, 0x00009253

// offsets for PDO entries
static unsigned int ctrl_word;
static unsigned int mode;
static unsigned int tar_torq;
static unsigned int max_torq;
static unsigned int tar_pos;

static unsigned int digital_output;
static unsigned int max_speed;
static unsigned int touch_probe_func;
static unsigned int tar_vel;
static unsigned int error_code;
static unsigned int status_word;
static unsigned int mode_display;
static unsigned int pos_act;
static unsigned int vel_act;
static unsigned int torq_act;
static unsigned int pos_gap_act;
static unsigned int touch_probe_status;
static unsigned int touch_probe_pos;
static unsigned int touch_probe_pos2;
static unsigned int digital_input;

bool running_thread = true;
bool isAllEnabled2 = false;

static struct
{
  unsigned int ctrl_word;
  unsigned int target_position;
  unsigned int target_torque;
  unsigned int target_velocity;
  unsigned int max_torque;
  unsigned int DO;
  unsigned int DI;
  unsigned int act_DI;

  unsigned int offset_velocity;
  unsigned int offset_torque;
  unsigned int offset_position;

  unsigned int error_code;
  unsigned int status_word;
  unsigned int act_position;
  unsigned int act_torque;

  unsigned int disp_mode;
  unsigned int actual_ferr;
  unsigned int probe_status;
  unsigned int probe1_pos;

  unsigned int probe2_pos;
  unsigned int act_velocity;
  unsigned int mode_Of_Operation;
  unsigned int mode_Of_Operation_dsiplay;
} offset2[E_STOP2 + 1];

struct {
  int defaultPosition = 0;
  int target_postion = 0;
  bool isInitedToDefault = false;
  bool isEnabled = false;
  uint16_t statusOld = 0;
  int statusDeCount = 5;
  int last_position = 0;
  int act_position = 0;
  int act_torque = 0;
  int act_velocity = 0;
  int act_wrong = 0;
} motorData2[E_STOP2 + 1];

const static ec_pdo_entry_reg_t domain2_regs[] = {
#if E_START2 <= 0 && E_STOP2 >= 0
    {0, 0, VidPid, 0x6040, 0, &offset2[0].ctrl_word},
    {0, 0, VidPid, 0x6071, 0, &offset2[0].target_torque},
    {0, 0, VidPid, 0x607a, 0, &offset2[0].target_position},
    {0, 0, VidPid, 0x60ff, 0, &offset2[0].target_velocity},

    {0, 0, VidPid, 0x6041, 0, &offset2[0].status_word},
    {0, 0, VidPid, 0x6064, 0, &offset2[0].act_position},
    {0, 0, VidPid, 0x606c, 0, &offset2[0].act_velocity},
    {0, 0, VidPid, 0x6077, 0, &offset2[0].act_torque},
    {0, 0, VidPid, 0x6061, 0, &offset2[0].mode_Of_Operation_dsiplay},
#endif
#if E_START2 <= 1 && E_STOP2 >= 1
    {0, 1, VidPid, 0x6040, 0, &offset2[1].ctrl_word},
    {0, 1, VidPid, 0x6071, 0, &offset2[1].target_torque},
    {0, 1, VidPid, 0x607a, 0, &offset2[1].target_position},
    {0, 1, VidPid, 0x60ff, 0, &offset2[1].target_velocity},

    {0, 1, VidPid, 0x6041, 0, &offset2[1].status_word},
    {0, 1, VidPid, 0x6064, 0, &offset2[1].act_position},
    {0, 1, VidPid, 0x606c, 0, &offset2[1].act_velocity},
    {0, 1, VidPid, 0x6077, 0, &offset2[1].act_torque},
    {0, 1, VidPid, 0x6061, 0, &offset2[1].mode_Of_Operation_dsiplay},
#endif
#if E_START2 <= 2 && E_STOP2 >= 2
    {0, 2, VidPid, 0x6040, 0, &offset2[2].ctrl_word},
    {0, 2, VidPid, 0x6071, 0, &offset2[2].target_torque},
    {0, 2, VidPid, 0x607a, 0, &offset2[2].target_position},
    {0, 2, VidPid, 0x60ff, 0, &offset2[2].target_velocity},

    {0, 2, VidPid, 0x6041, 0, &offset2[2].status_word},
    {0, 2, VidPid, 0x6064, 0, &offset2[2].act_position},
    {0, 2, VidPid, 0x606c, 0, &offset2[2].act_velocity},
    {0, 2, VidPid, 0x6077, 0, &offset2[2].act_torque},
    {0, 2, VidPid, 0x6061, 0, &offset2[2].mode_Of_Operation_dsiplay},
#endif
#if E_START2 <= 3 && E_STOP2 >= 3
    {0, 3, VidPid, 0x6040, 0, &offset2[3].ctrl_word},
    {0, 3, VidPid, 0x6071, 0, &offset2[3].target_torque},
    {0, 3, VidPid, 0x607a, 0, &offset2[3].target_position},
    {0, 3, VidPid, 0x60ff, 0, &offset2[3].target_velocity},

    {0, 3, VidPid, 0x6041, 0, &offset2[3].status_word},
    {0, 3, VidPid, 0x6064, 0, &offset2[3].act_position},
    {0, 3, VidPid, 0x606c, 0, &offset2[3].act_velocity},
    {0, 3, VidPid, 0x6077, 0, &offset2[3].act_torque},
    {0, 3, VidPid, 0x6061, 0, &offset2[3].mode_Of_Operation_dsiplay},
#endif
#if E_START2 <= 4 && E_STOP2 >= 4
    {0, 4, VidPid, 0x6040, 0, &offset2[4].ctrl_word},
    {0, 4, VidPid, 0x6071, 0, &offset2[4].target_torque},
    {0, 4, VidPid, 0x607a, 0, &offset2[4].target_position},
    {0, 4, VidPid, 0x60ff, 0, &offset2[4].target_velocity},

    {0, 4, VidPid, 0x6041, 0, &offset2[4].status_word},
    {0, 4, VidPid, 0x6064, 0, &offset2[4].act_position},
    {0, 4, VidPid, 0x606c, 0, &offset2[4].act_velocity},
    {0, 4, VidPid, 0x6077, 0, &offset2[4].act_torque},
    {0, 4, VidPid, 0x6061, 0, &offset2[4].mode_Of_Operation_dsiplay},
#endif
#if E_START2 <= 5 && E_STOP2 >= 5
    {0, 5, VidPid, 0x6040, 0, &offset2[5].ctrl_word},
    {0, 5, VidPid, 0x6071, 0, &offset2[5].target_torque},
    {0, 5, VidPid, 0x607a, 0, &offset2[5].target_position},
    {0, 5, VidPid, 0x60ff, 0, &offset2[5].target_velocity},

    {0, 5, VidPid, 0x6041, 0, &offset2[5].status_word},
    {0, 5, VidPid, 0x6064, 0, &offset2[5].act_position},
    {0, 5, VidPid, 0x606c, 0, &offset2[5].act_velocity},
    {0, 5, VidPid, 0x6077, 0, &offset2[5].act_torque},
    {0, 5, VidPid, 0x6061, 0, &offset2[5].mode_Of_Operation_dsiplay},
#endif
#if E_START2 <= 6 && E_STOP2 >= 6
    {0, 6, VidPid, 0x6040, 0, &offset2[6].ctrl_word},
    {0, 6, VidPid, 0x6071, 0, &offset2[6].target_torque},
    {0, 6, VidPid, 0x607a, 0, &offset2[6].target_position},
    {0, 6, VidPid, 0x60ff, 0, &offset2[6].target_velocity},

    {0, 6, VidPid, 0x6041, 0, &offset2[6].status_word},
    {0, 6, VidPid, 0x6064, 0, &offset2[6].act_position},
    {0, 6, VidPid, 0x606c, 0, &offset2[6].act_velocity},
    {0, 6, VidPid, 0x6077, 0, &offset2[6].act_torque},
    {0, 6, VidPid, 0x6061, 0, &offset2[6].mode_Of_Operation_dsiplay},
#endif

    {}};

// 伺服电机的PDO映射参数
ec_pdo_entry_info_t Igh_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6071, 0x00, 16},
    {0x607a, 0x00, 32},
    {0x60ff, 0x00, 32},

    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x6061, 0x00, 8},
};

// 伺服电机PDO映射参数的组地址
ec_pdo_info_t Igh_pdos[] = {
    {0x1600, 4, Igh_pdo_entries + 0},
    {0x1a00, 5, Igh_pdo_entries + 4},
};

ec_sync_info_t Igh_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, Igh_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, Igh_pdos + 1, EC_WD_DISABLE},
    {0xFF}};

/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2) {
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
    result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
  } else {
    result.tv_sec = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }

  return result;
}

/*****************************************************************************/

void Igh_rechekTime2() {
  clock_gettime(CLOCK_TO_USE, &apptime);

  ecrt_master_application_time(master2, TIMESPEC2NS(apptime));
  if (sync_ref_counter2) {
    sync_ref_counter2--;
  } else {
    sync_ref_counter2 = 1;                      // sync every cycle
    ecrt_master_sync_reference_clock(master2);  // DC reference clock drift compensation
  }
  ecrt_master_sync_slave_clocks(master2);  // DC clock drift compensation
  // queue process data
  ecrt_domain_queue(domain2);
  // send EtherCAT data
  ecrt_master_send(master2);
}

void readWriteStatus2() {
  int i2;
  isAllEnabled2 = true;
  ecrt_master_receive(master2);
  ecrt_domain_process(domain2);

  for (i2 = E_START2; i2 <= E_STOP2; i2++) {
    uint16_t ss = EC_READ_U16(domain2_pd + offset2[i2].status_word);
    if (motorData2[i2].statusOld != ss) {
      if ((ss == 0x1237 && motorData2[i2].statusOld == 0x0237) || (ss == 0x0237 && motorData2[i2].statusOld == 0x1237)) {
        // NO PRINT
      } else {
        printf("status2 %d : 0x%04x to 0x%04x  \n", i2, motorData2[i2].statusOld, ss);
      }

      motorData2[i2].statusOld = ss;
      motorData2[i2].statusDeCount = 5;
    }

    // 电机未使能,则执行状态机切换
    if ((ss & 0xFF) != 0x37) {
      motorData2[i2].isEnabled = false;
      if (motorData2[i2].statusDeCount == 5) {
        if ((ss & 0xFF) == 0x50) {
          EC_WRITE_U16(domain2_pd + offset2[i2].ctrl_word, 0x06);
        } else if ((ss & 0xFF) == 0x31) {
          motorData2[i2].last_position = EC_READ_S32(domain2_pd + offset2[i2].act_position);
          motorData2[i2].target_postion = motorData2[i2].last_position;
          EC_WRITE_S32(domain2_pd + offset2[i2].target_position, motorData2[i2].last_position);
          EC_WRITE_U16(domain2_pd + offset2[i2].ctrl_word, 0x07);
        } else if ((ss & 0xFF) == 0x21) {
          motorData2[i2].last_position = EC_READ_S32(domain2_pd + offset2[i2].act_position);
          motorData2[i2].target_postion = motorData2[i2].last_position;
          EC_WRITE_S32(domain2_pd + offset2[i2].target_position, motorData2[i2].last_position);
          EC_WRITE_U16(domain2_pd + offset2[i2].ctrl_word, 0x07);
        } else if ((ss & 0xFF) == 0x33) {
          motorData2[i2].last_position = EC_READ_S32(domain2_pd + offset2[i2].act_position);
          motorData2[i2].target_postion = motorData2[i2].last_position;
          EC_WRITE_S32(domain2_pd + offset2[i2].target_position, motorData2[i2].last_position);
          motorData2[i2].target_postion = motorData2[i2].last_position;
          EC_WRITE_U16(domain2_pd + offset2[i2].ctrl_word, 0x0F);
        }
      }
      if (motorData2[i2].statusDeCount <= 0) {
        motorData2[i2].statusDeCount = 5;
      } else {
        motorData2[i2].statusDeCount--;
      }
    } else {
      motorData2[i2].isEnabled = true;
    }
  }

  // 判断所有电机都使能
  for (i2 = E_START2; i2 <= E_STOP2; i2++) {
    isAllEnabled2 = (isAllEnabled2 && motorData2[i2].isEnabled);
  }
}

void readWritePDO2() {
  int i2 = 0;

  ecrt_master_receive(master2);
  ecrt_domain_process(domain2);

  for (i2 = E_START2; i2 <= E_STOP2; i2++) {
    uint16_t ss = EC_READ_U16(domain2_pd + offset2[i2].status_word);
    if ((ss & 0xFF) == 0x37) {
      motorData2[i2].act_position = EC_READ_S32(domain2_pd + offset2[i2].act_position);
      motorData2[i2].last_position = motorData2[i2].act_position;
      motorData2[i2].act_velocity = EC_READ_S32(domain2_pd + offset2[i2].act_velocity);
      motorData2[i2].act_torque = EC_READ_S16(domain2_pd + offset2[i2].act_torque);
      if (motorData2[i2].isInitedToDefault == false) {
        if (motorData2[i2].last_position > motorData2[i2].defaultPosition + 80) {
          motorData2[i2].target_postion = motorData2[i2].target_postion - 120;
        } else if (motorData2[i2].last_position < motorData2[i2].defaultPosition - 80) {
          motorData2[i2].target_postion = motorData2[i2].target_postion + 120;
        } else {
          motorData2[i2].isInitedToDefault = true;
        }
        printf("position2 + %d,  %d to %d  \n", i2, motorData2[i2].last_position, motorData2[i2].target_postion);
        EC_WRITE_S32(domain2_pd + offset2[i2].target_position, motorData2[i2].target_postion);
      }
    } else {
      printf("wrong status %d  \n", ss);
    }
  }
}

/**
 * 主时钟循环程序,要每8毫秒执行一次,
 * 功能为,切换电机的状态机,读取上层应用下发的位置命令,写入电机的下发位置,
 * 读取电机当前的 位置 速度 扭力 ，反馈给上层应用
 * **/
void* rt_thread_function(void* arg) {
  // if(2>1)return 0;
  struct timespec wakeupTime, time;
  // get current time
  clock_gettime(CLOCK_TO_USE, &wakeupTime);

  while (running_thread) {
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

    if (!isAllEnabled2) {
      readWriteStatus2();
    } else {
      readWritePDO2();
    }

    Igh_rechekTime2();
  }
  return NULL;
}

void Igh_master_activate2() {
  printf("......Activating master2......\n");
  if (ecrt_master_activate(master2)) {
    exit(EXIT_FAILURE);
  }
  if (!(domain2_pd = ecrt_domain_data(domain2))) {
    exit(EXIT_FAILURE);
  }

  printf("......Master2  Activated.......\n");
}

/****************************************************************************/

void Igh_init2() {
  ec_master_info_t master_info;
  ec_slave_info_t slave_info;
  int ret;
  int slavecnt;
  ec_slave_config_t* sc;

  // uint32_t  abort_code;
  // size_t rb;
  int ii = 0;
  master2 = ecrt_request_master(1);
  if (!master2) {
    exit(EXIT_FAILURE);
  }

  domain2 = ecrt_master_create_domain(master2);
  if (!domain2) {
    exit(EXIT_FAILURE);
  }

  //---------get master / salve info----------------------
  ret = ecrt_master(master2, &master_info);
  slavecnt = master_info.slave_count;
  printf("ret = %d, slavecnt = %d, apptime = %" PRIu64 "\n", ret, master_info.slave_count, master_info.app_time);
  // ret = ecrt_master_get_slave(master2, 0, &slave_info);
  // printf("ret = %d,spos = %d, pcode = %d\n", ret, slave_info.position, slave_info.product_code);

  //---------end get master / salve info----------------------

  for (ii = E_START2; ii <= E_STOP2; ii++) {
    printf("servo %d  begin init! \n", ii);
    ec_slave_config_t* sc;
    if (!(sc = ecrt_master_slave_config(master2, 0, ii, VidPid))) {
      fprintf(stderr, "Failed to get slave configuration for Igh.\n");
      exit(EXIT_FAILURE);
    }

    printf("Configuring PDOs... %d \n", ii);
    if (ecrt_slave_config_pdos(sc, EC_END, Igh_syncs)) {
      fprintf(stderr, "Failed to configure Igh PDOs.\n");
      exit(EXIT_FAILURE);
    }

    ecrt_slave_config_sdo16(sc, 0x6040, 0, 7);  // 下使能
    sleep(1);
    ecrt_slave_config_sdo16(sc, 0x6040, 0, 6);  // 下使能
    sleep(1);    
    ecrt_slave_config_sdo16(sc, 0x6040, 0, 128);// 清错
    sleep(1);

    ecrt_slave_config_sdo16(sc, 0x1C32, 1, 2);  // set output synchronization triggered by  sync0 event DC mode
    ecrt_slave_config_sdo16(sc, 0x1C33, 1, 2);  // set input  synchronization triggered by  sync1 evnent DC mode
    ecrt_slave_config_sdo8(sc, 0x60c2, 1, (1000 / TASK_FREQUENCY));
    ecrt_slave_config_sdo8(sc, 0x60c2, 2, -3);
    ecrt_slave_config_sdo8(sc, 0x6060, 0, 8);
    ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, PERIOD_NS / 2, 0, 0);
  }

  if (ecrt_domain_reg_pdo_entry_list(domain2, domain2_regs)) {
    fprintf(stderr, "PDO entry registration failed!\n");
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char** argv) {
  Igh_init2();
  Igh_master_activate2();

  pthread_t rt_thread;
  pthread_attr_t attr;
  struct sched_param param;

  // Initialize thread attributes
  if (pthread_attr_init(&attr) != 0) {
    perror("pthread_attr_init");
    exit(EXIT_FAILURE);
  }

  // Set scheduling policy to SCHED_FIFO
  if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0) {
    perror("pthread_attr_setschedpolicy");
    exit(EXIT_FAILURE);
  }

  // Set thread priority
  param.sched_priority = THREAD_PRIORITY;
  if (pthread_attr_setschedparam(&attr, &param) != 0) {
    perror("pthread_attr_setschedparam");
    exit(EXIT_FAILURE);
  }

  // Make the scheduling parameters take effect
  if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0) {
    perror("pthread_attr_setinheritsched");
    exit(EXIT_FAILURE);
  }

  // Create real-time thread
  if (pthread_create(&rt_thread, &attr, rt_thread_function, NULL) != 0) {
    printf("pthread_create\n");
    exit(EXIT_FAILURE);
  }

  // Wait for the thread to complete
  if (pthread_join(rt_thread, NULL) != 0) {
    perror("pthread_join");
    exit(EXIT_FAILURE);
  }

  // Destroy thread attributes
  if (pthread_attr_destroy(&attr) != 0) {
    perror("pthread_attr_destroy");
    exit(EXIT_FAILURE);
  }

  return 0;
}

/****************************************************************************/
