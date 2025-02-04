#ifndef ETHERCAT_INTERFACE_R_H
#define ETHERCAT_INTERFACE_R_H

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
#include <inttypes.h>
#include <math.h>
#include "joint_data.h"

#define TASK_FREQUENCY 250 /* Hz */
#define E_START 2
#define E_STOP 8

#define VidPid 0x00522227, 0x00009253

extern bool running_thread;
extern bool isAllEnabled;
extern int action_value_;
extern uint8_t* domain1_pd;
extern JointData rightArm;

void Ethercat_syncThread();
void Igh_master_activate();
void Igh_init();
void toDefaultPositions();
void toPositionsByQueue();
void toEnable();
void consoleJoints();
void consoleQueue();
void Igh_rechekTime();
struct timespec timespec_add(struct timespec time1, struct timespec time2);

#endif // ETHERCAT_INTERFACE_R_H
