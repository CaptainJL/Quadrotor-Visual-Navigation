#include "gpu_w_flow.hpp"



using namespace std;
using namespace flowfilter;
using namespace flowfilter::gpu;




__global__ void wflow_K(gpuimage_t<float2> flow, gpuimage_t<float3> sflow, WFlow_Params params)
{
	int2 size = 	make_int2(flow.width,flow.height);
	int2 pix = 		make_int2(blockIdx.x*blockDim.x+threadIdx.x,
							blockIdx.y*blockDim.y+threadIdx.y);
	if(pix.x>=size.x || pix.y>=size.y){return;}

  	// Access
  	float2 flow_pix = 	*coordPitch(flow, pix);
	double flowX = 		(double)(flow_pix.x);
	double flowY = 		(double)(flow_pix.y);

	// Simplifiers
	double xd =			pix.x-params.x0;
	double yd =			pix.y-params.y0;
	double xymag = 		sqrt( xd*xd + yd*yd );
	double hypmag =		sqrt( xymag*xymag + params.f*params.f);

	// Structure Flow
  	double3 sflow_p = 	make_double3(0.0, 0.0, 0.0);

  	// printf("sfx = %0.3f\n", (float)(sflow_p.x));


	if( xymag>=10 && xymag<=params.rad )
	{
		double theta =	atan2(xymag,params.f);
		double S_area = pow(cos(theta),3)/(params.f*params.f);

		sflow_p.x = S_area/params.S_cap*params.f/xymag*flowX;
		sflow_p.y = S_area/params.S_cap*params.f/xymag*flowY;
		sflow_p.z = -S_area/params.S_cap*params.f/xymag*tan(theta)*(flowX*xd/xymag + flowY*yd/xymag);

		/*if ( (pix.x==170) && (pix.y==100) )
		{ 
			printf("sf = %0.3f, %0.3f, %0.3f\n", sflow_p.x, sflow_p.y, sflow_p.z); 
			printf("%0.3f, %0.3f, %0.3f\n", params.f, params.S_cap, theta);
		}*/

	}
	else
	{
		sflow_p.x = 0.0;
		sflow_p.y = 0.0;
		sflow_p.z = 0.0;
	}


  	float3 sflow_pix =			make_float3((float)(sflow_p.x), (float)(sflow_p.y), (float)(sflow_p.z));
	*coordPitch(sflow, pix) = 	sflow_pix;
}






void configureKernelGrid(const int height, const int width, const dim3 block, dim3& grid) 
{
    float w = width;
    float h = height;
    float x = block.x;
    float y = block.y;

    grid.x = (int)ceilf(w / x);
    grid.y = (int)ceilf(h / y);
    grid.z = 1;
}



WFlow_GPU::WFlow_GPU()
{
  	
  	

}
WFlow_GPU::~WFlow_GPU()
{

}






















/* OLD VERSION
__global__ void wflow_K(gpuimage_t<float2> flow, gpuimage_t<float3> sflow, WFlow_Params params)
{
	int2 size = 	make_int2(flow.width,flow.height);
	int2 pix = 		make_int2(blockIdx.x*blockDim.x+threadIdx.x,
							blockIdx.y*blockDim.y+threadIdx.y);
	if(pix.x>=size.x || pix.y>=size.y){return;}

  	// Access
  	float2 flow_pix = 	*coordPitch(flow, pix);
	double flowX = 		(double)(flow_pix.x);
	double flowY = 		(double)(flow_pix.y);

	// Simplifiers
	double xd =			abs(pix.x-params.x0);
	double xd2 =		xd*xd;
	double yd =			abs(pix.y-params.y0);
	double yd2 =		yd*yd;
	double f2 = 		params.f*params.f;
	double pl2 =		params.pix_len*params.pix_len;
	double dist = 		sqrt( (xd)*(xd) + (yd)*(yd) );

	// Structure Flow
  	double3 sflow_p = 	make_double3(0.0, 0.0, 0.0);

  	// printf("sfx = %0.3f\n", (float)(sflow_p.x));


	if( 78.0<=dist && dist<=params.rad )
	{
		sflow_p.x = -(1.0/(f2)*(pl2)*(flowX*(xd2/(xd2+yd2+f2)-1.0)*1.0/sqrt(xd2+yd2+f2)+flowY*(xd)*(yd)*1.0/pow(xd2+yd2+f2,3.0/2.0))*1.0/pow(1.0/(f2)*(xd2+yd2)+1.0,3.0/2.0))/params.S_cap;
	   	sflow_p.y = -(1.0/(f2)*(pl2)*1.0/pow(1.0/(f2)*(xd2+yd2)+1.0,3.0/2.0)*(flowY*(yd2/(xd2+yd2+f2)-1.0)*1.0/sqrt(xd2+yd2+f2)+flowX*(xd)*(yd)*1.0/pow(xd2+yd2+f2,3.0/2.0)))/params.S_cap;
	   	sflow_p.z = -(1.0/(f2)*(pl2)*1.0/pow(1.0/(f2)*(xd2+yd2)+1.0,3.0/2.0)*(params.f*flowX*(xd)*1.0/pow(xd2+yd2+f2,3.0/2.0)+params.f*flowY*(yd)*1.0/pow(xd2+yd2+f2,3.0/2.0)))/params.S_cap;		
	}
	else
	{
		sflow_p.x = 0.0;
		sflow_p.y = 0.0;
		sflow_p.z = 0.0;
	}



	// if ( (pix.x==50) && (pix.y==50) )
	// { printf("sfx = %0.3f\n", (float)(sflow_p.x)); }

  	float3 sflow_pix =			make_float3((float)(sflow_p.x), (float)(sflow_p.y), (float)(sflow_p.z));
	*coordPitch(sflow, pix) = 	sflow_pix;
}
*/











void 
WFlow_GPU::calc_wflow()
{
	__block = dim3(16, 16, 1);
	configureKernelGrid((int)wf_params.cam_h, (int)wf_params.cam_w, __block, __grid);

	wflow_K<<<__grid, __block>>>(flow_gpu.wrap<float2>(), sflow_gpu.wrap<float3>(), wf_params);

}
