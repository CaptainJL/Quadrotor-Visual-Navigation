#ifndef FC_H_
#define FC_H_

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <vector>
#include <string>
#include <apps/Common/exampleHelper.h>
#include <common/minmax.h>
#include <thread>
#include <mutex>
#include <exception>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <GPU_2_Quad_ROS/to_px4.h>
#include <GPU_2_Quad_ROS/from_px4.h>

#include <flowfilter/gpu/image.h>
#include <flowfilter/image.h>
#include <flowfilter/gpu/flowfilter.h>
#include <flowfilter/gpu/display.h>

#include "gpu_w_flow.hpp"

#define PI 3.14159f

class Flow_Control
{
public:
	// Constructor/Deconstructor	
	Flow_Control();
	~Flow_Control();

	// Multithreading Threads + Lock
  	void loop_flow();
	void loop();
	void loop_display();
	void loop_wzspeed();

	// Multithreading
	std::thread spawn_flow()
  	{  	return std::thread( [=] { loop_flow(); } );	}

	std::thread spawn()
  	{  	return std::thread( [=] { loop(); } ); 	}

	std::thread spawn_display()
  	{  	return std::thread( [=] { loop_display(); } ); 	}
	
	std::thread spawn_wzspeed()
  	{  	return std::thread( [=] { loop_wzspeed(); } ); 	}

	std::mutex lockThread;

	// Externally Referenced Image
	cv::Mat imageCV;

	// Initialisers
	void init();

	// Counters and Flags
	bool start;
	int img_ctr, flow_ctr, ctrl_ctr, disp_ctr;



private:

	// Image Wrapper
	void _wrapCVMat(cv::Mat&, flowfilter::image_t&);

	// ROS IMU callback
	void _imu_callback(const GPU_2_Quad_ROS::from_px4);
	
	// Camera Specs
	float cam_specs[5];	// rate, fx, fy, xo, yo
	float cam_h, cam_w;

	// Vehicle State
	float vel[3];	// vx, vy, vz
	float rpy[3];	// roll, pitch, yaw	
	float vmap[3];	// mag, xref, yref
	float gains[2];

	// Timing
	double start_time;

	// ROS Components
	ros::NodeHandle 			node;
	ros::Subscriber 			px4refsub;
	ros::Publisher 				px4cmdpub;
	GPU_2_Quad_ROS::from_px4 	px4data;
	GPU_2_Quad_ROS::to_px4 		px4cmd;

	// Flow Components
	float maxflow;
	flowfilter::gpu::PyramidalFlowFilter filter; 
    flowfilter::gpu::FlowToColor flowColor;


	// Image/Containers
	cv::Mat imageFCV;		flowfilter::image_t imageF;
	cv::Mat imageDCV;		
	cv::Mat flowCV;			flowfilter::image_t flow;
 	cv::Mat	flowCtrlCV;		flowfilter::image_t flowCtrl;
  	cv::Mat	flowDispCV;	
	cv::Mat	ctrlCostsCV;
	cv::Mat	ctrlCostsDCV;
	// for transport:
	cv::Mat	flowTransCV;
	cv::Mat	ctrlCostsTransCV;

	cv::Mat sflowCV;			flowfilter::image_t sflow;

	cv::Mat saveDataCV;

	// Control wz flow copy
	cv::Mat wzflowCV;


	// Functions
	void _flow_comp();
	void _vel_map();
	void _costs_comp();
	void _ctrl_comp(double);
	void _hover_comp_gpu(double);
	void _hover_comp(double);
	
	float _cost_px(int2, float2);
	double wz_speed_comp();

	// Values for use
	float yaw_ref;


	// velocity references
	float x_vel, y_vel;


	// Vel controller integrals
	float xint, yint;
	float dX, dY;

	// Saving outputs
	std::ofstream flowtimefile;
	std::ofstream ctrltimefile;
	std::ofstream flowfile;
	std::ofstream ctrlcostsfile;

	// Baro + Yaw setpoints
	bool home_set;
	float baro_home;
	float yaw_home;


	// Hover control vars
	float hover_xInt, hover_yInt, hover_zInt;
	double dt, time_p, time_n;


	// Hover GPU
	WFlow_GPU wf_gpu;


	// Ctrl 
	double epsilon_p, sigma_p;
	double xd_p;
	double ang_vel;


	// Current mode - section
	int mode_use;
	int mode_old;
	int mode_direct;

	// hatLz filter
	double hatLz;
 	float wzflow_n;

	// wz speed
	double wzspeed;
	double vwz, dvwz;


	struct CostMap
	{
		float alpha;
		float k;
		float B;
		float g0;
		float g1;
	} cmap;



};
























#endif
