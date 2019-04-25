#ifndef _GPU_WFLOW_H
#define _GPU_WFLOW_H


#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <string>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>

#include <cuda.h>
#include <cuda_runtime.h>

#include <flowfilter/gpu/image.h>
#include <flowfilter/image.h>
#include <flowfilter/gpu/flowfilter.h>
#include <flowfilter/gpu/display.h>
#include <flowfilter/gpu/image.h>
#include <flowfilter/gpu/device/image_k.h>






struct WFlow_Params
{
	// Camera Params
	double cam_h, cam_w;
	double x0,y0;
	double f;
	double pix_len;
	double freq;

	double L0, L1, L2;		// Lambda 

	double rad;
	double S_cap;			// Spherical Cap
};







class WFlow_GPU
{
public:

	WFlow_GPU();
	~WFlow_GPU();


	flowfilter::gpu::GPUImage flow_gpu;
	flowfilter::gpu::GPUImage sflow_gpu;

	void calc_wflow();

	WFlow_Params wf_params;



	dim3 __block, __grid;




};












#endif