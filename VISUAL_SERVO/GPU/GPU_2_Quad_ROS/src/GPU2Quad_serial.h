#ifndef GPU2QUAD_SERIAL_H_
#define GPU2QUAD_SERIAL_H_

// STANDARD + SERIAL
#include <stdio.h> // standard input / output functions
#include <stdlib.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/time.h>
#include <signal.h>

// ROS
#include "ros/ros.h"
#include <GPU_2_Quad_ROS/to_px4.h>
#include <GPU_2_Quad_ROS/from_px4.h>
//#include "../msg_gen/cpp/include/GPU_2_Quad_ROS/to_px4.h"
//#include "../msg_gen/cpp/include/GPU_2_Quad_ROS/from_px4.h"
#include <std_msgs/String.h>

#define TELEM_FREQ 1
#define BAUD_RATE B921600
//#define true 1
//#define false 0

//using namespace std;

// Serial Characteristics
const char* port_name = "/dev/ttyTHS2";
int port_id;
bool port_status = false;
bool is_read_next = true;	// Must recieve command before operation

// Serial from Quad data
uint8_t serial_in[48];

struct termios config;      // structure to store the port settings in

// Quadrotor Current States
float sh, avg, avg_calc;
float x_vel, y_vel, z_vel;
//float roll_pos, pitch_pos, yaw_pos;
float roll, pitch, yaw;
float roll_rate, pitch_rate, yaw_rate;
float barometer;

// GPU Output Desired States
float x_vel_des, y_vel_des, z_vel_des;
float flow_vbx, flow_vby, flow_viz;
float mode_sel, yaw_des;

// Package Header
uint32_t serial_header = 0x48234567;

// Time
struct timeval before , after;

// Correct
int count_read, correct_read;



// ROS_PARTS





















#endif
