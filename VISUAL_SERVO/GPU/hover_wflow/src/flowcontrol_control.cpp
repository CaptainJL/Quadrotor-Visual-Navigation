
#include "flowcontrol_h.hpp"


// Relevant Namespaces used
using namespace std;
using namespace cv;
using namespace flowfilter;
using namespace flowfilter::gpu;
using namespace Eigen;









// Flow Comp   
void
Flow_Control::
_flow_comp()
{
	// Download image and upload to GPU
	lockThread.lock();
	imageFCV = imageCV.clone();
	lockThread.unlock();
	_wrapCVMat(imageFCV, imageF);

	// Upload to GPU and calculate Flow
	filter.loadImage(imageF);
  	filter.compute();
	// Download flow	
	_wrapCVMat(flowCV, flow);
	filter.downloadFlow(flow);
	lockThread.lock();
	flowTransCV = flowCV.clone();
	lockThread.unlock();

	//flowtimefile << filter.elapsedTime() <<endl;
}





// Yanni Hover
void
Flow_Control::
_hover_comp_gpu(double dt)
{
	// Download flow
	lockThread.lock();
	flowCtrlCV = flowTransCV.clone();
	lockThread.unlock();

	_wrapCVMat(flowCtrlCV, flowCtrl);

	// calc on gpu and download
	wf_gpu.flow_gpu.upload(flowCtrl);
	wf_gpu.calc_wflow();
	wf_gpu.sflow_gpu.download(sflow);

	// w flow calc
	float sumflowX = 	(float)wf_gpu.wf_params.freq/2*sum(sflowCV)[0];
	float sumflowY = 	(float)wf_gpu.wf_params.freq/2*sum(sflowCV)[1];
	float sumflowZ = 	(float)wf_gpu.wf_params.freq/2*sum(sflowCV)[2];
	float wx = 			7.0f/1000.0f*sumflowY/(float)wf_gpu.wf_params.L0;	// 2.5f
	float wy = 			-7.0f/1000.0f*sumflowX/(float)wf_gpu.wf_params.L1;	// -2.5f
	float wz = 			-8.0f/1000.0f*sumflowZ/(float)wf_gpu.wf_params.L2; 	// -3.0f
	//printf("vel w: %0.3f %0.3f %0.3f\n",wx,wy,wz);

	// Position Integrals
	if(abs(wx)>0.02f){ hover_xInt += (float)dt*wx; }
	if(abs(wy)>0.02f){ hover_yInt += (float)dt*wy; }
	//if(abs(wz)>0.01f){ hover_zInt += (float)dt*wz; }


	// Calculate controller
	float xy_pgain = 0.25f;
	float z_pgain = 0.8f;

	//float ctrl_vbx = -xy_pgain*hover_xInt;
	//float ctrl_vby = -xy_pgain*hover_yInt; 
	//float ctrl_viz = -z_pgain*(hover_zInt - -2.0f); // 2m hold

	
	//  hatLz Filter
	double dhatLz = -wz;
	hatLz +=		dt*dhatLz ;
	double z_wz =	-exp(hatLz); 


	// use baro alt
	//float ctrl_viz = -z_pgain*(px4data.baro-(baro_home-1.5f));
	float ctrl_viz = 0;
	float ctrl_vbx = -0.3f*wx;
	float ctrl_vby = -0.3f*wy;

	float x0 = 				752.0f/2.0f;
	float y0 =				480.0f/2.0f;
	float f = 				235.0f; //376.0f; //235.0f; //200.0f;

	if (abs(wz)>1.3f)	// Stops aggressive sideways takeoff or landing
	{ 
		wx = 0;
		wy = 0;
	}

	if (mode_direct==2 || mode_direct==4)
	{
		hatLz = log(2.0);
		ctrl_viz = 0;
		ctrl_vbx = -xy_pgain*(hover_xInt);
		ctrl_vby = -xy_pgain*(hover_yInt); 
	}
	else if (mode_direct==3)
	{
		ctrl_viz = -z_pgain*(z_wz-(-2.0f));
		ctrl_vbx = -xy_pgain*(hover_xInt-vs_x_des);
		ctrl_vby = -xy_pgain*(hover_yInt-vs_y_des); 
		//printf("intx,y: %0.3f, %0.3f\n", hover_xInt, hover_yInt);
	}
	else
	{
		hatLz = log(2.0);	// think its at 1.5m for stabilisation of height
		hover_xInt = 0;
		hover_yInt = 0;
	}

	


	if (visservo_sx>0 && visservo_sy>0 && mode_direct!=0)
	{
		ctrl_vbx = 		visservo_xcmd;
		ctrl_vby = 		visservo_ycmd;

		//cout << "using visservo" << endl;
	}

	// Publish
	px4cmd.x_vel_des = 	ctrl_vbx;
	px4cmd.y_vel_des = 	ctrl_vby;
	px4cmd.z_vel_des = 	ctrl_viz;
	px4cmd.vbx =		wx;
	px4cmd.vby =		wy;
	px4cmd.viz =		z_wz; // z_wz alternative
	px4cmd.mode = 		-5.0f; // hover
	px4cmd.yaw_des = 	yaw_home;

	//printf("xpos, ypos: %0.3f, %0.3f\n", hover_xInt, hover_yInt);
	//printf("wflow: %0.3f, %0.3f, %0.3f\n", wx, wy,wz);

}











