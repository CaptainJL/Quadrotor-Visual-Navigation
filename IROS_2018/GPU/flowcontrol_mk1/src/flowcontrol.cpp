#include "flowcontrol_h.hpp"


// Relevant Namespaces used
using namespace std;
using namespace cv;
using namespace flowfilter;
using namespace flowfilter::gpu;
using namespace Eigen;



// Constructor/Deconstructor	
Flow_Control::
Flow_Control()
{
	img_ctr = 	0; 
	flow_ctr = 	0; 
	ctrl_ctr = 	0;
	disp_ctr = 	0;

	home_set = false;

	px4refsub = node.subscribe<GPU_2_Quad_ROS::from_px4>("px4_from", 10, &Flow_Control::_imu_callback, this);
	px4cmdpub = node.advertise<GPU_2_Quad_ROS::to_px4>("px4_to", 10);

	start_time = (double)ros::Time::now().toSec();
	//flowtimefile.open("/home/nvidia/ros_ws/flowcontrol_mk1/output/flowtime.csv");
	//ctrltimefile.open("/home/nvidia/ros_ws/flowcontrol_mk1/output/ctrltime.csv");
}
Flow_Control::
~Flow_Control()
{
}


// Initialisation
void
Flow_Control::
init()
{
	// Camera Sepcs
	cam_specs[0] = 	120;
	cam_specs[1] = 	220;	// 240
	cam_specs[2] = 	220;
	cam_specs[3] = 	187.5;
	cam_specs[4] = 	119.5;
  	cam_h = 		imageCV.rows;
  	cam_w = 		imageCV.cols;
	
  	// Image Structures
  	flowCV =		Mat(cam_h, cam_w, CV_32FC2);	_wrapCVMat(flowCV, flow);
 	flowCtrlCV = 	Mat(cam_h, cam_w, CV_32FC2);	_wrapCVMat(flowCtrlCV, flowCtrl);
  	flowDispCV = 	Mat(cam_h, cam_w, CV_32FC2);	
	// Cost structures and control speed
	ctrlCostsCV = 	Mat::zeros(cam_h, cam_w, CV_32FC1);	
	wzflowCV = 		Mat(cam_h, cam_w, CV_32FC2);
	// hoverRefCV = 	Mat(cam_h, cam_w, CV_32FC3);	
	// transport
	flowTransCV = 	Mat(cam_h, cam_w, CV_32FC2);
	ctrlCostsTransCV = 	Mat(cam_h, cam_w, CV_32FC1);
	// save data
	saveDataCV =	Mat(cam_h, cam_w, CV_8UC3);
	// Hover
	sflowCV =		Mat(cam_h, cam_w, CV_32FC3); 	_wrapCVMat(sflowCV, sflow);
	wf_gpu.sflow_gpu.upload(sflow);

	// Flow Characteristics
  	maxflow = 4.0f;
  	vector<float> gamma = {250.0f,100.0f};
  	vector<int> smoothIterations = {2,2};
  	filter = PyramidalFlowFilter(cam_h, cam_w, 2);
  	filter.setMaxFlow(maxflow);
  	filter.setGamma(gamma);
  	filter.setSmoothIterations(smoothIterations);





	// GPU alloc
	wf_gpu.wf_params.cam_h = 	cam_h;
	wf_gpu.wf_params.cam_w = 	cam_w;
	wf_gpu.wf_params.x0 =		(double)cam_w/2.0;
	wf_gpu.wf_params.y0 =		(double)cam_h/2.0;
	wf_gpu.wf_params.f = 		cam_specs[1];	
	wf_gpu.wf_params.pix_len =	1.0;
	wf_gpu.wf_params.freq =		(double)cam_specs[0];
	wf_gpu.wf_params.rad = 		115.0;

	double theta_0 = 	atan2(wf_gpu.wf_params.rad,wf_gpu.wf_params.f);
	double lambda =		(sin(theta_0)*sin(theta_0)) / (4.0-sin(theta_0)*sin(theta_0));
	double Lambda_r =	PI*pow(sin(theta_0),4)/4.0;

	wf_gpu.wf_params.L0 =		Lambda_r/lambda;
	wf_gpu.wf_params.L1 =		Lambda_r/lambda;
	wf_gpu.wf_params.L2 =		Lambda_r*2.0;
	wf_gpu.wf_params.S_cap = 	2.0*PI*(1.0-cos(theta_0));


	// hatLz filter
	hatLz = log(0.1);

	wzspeed = 0;
	vwz = 0;


	// Cost Mapping vars
	cmap.B = 		10.0f;
	cmap.g0 =		1.0f;
	cmap.g1 =		30.0f;
	cmap.alpha =	logf(cmap.g1/cmap.g0);
	cmap.k =		cmap.B/(cmap.g1/cmap.g0-1.0f);


}
// Image Wrapper
void
Flow_Control::
_wrapCVMat(Mat& cvMat_ref, image_t& img_ref) {

	img_ref.height =   cvMat_ref.rows;
    img_ref.width =    cvMat_ref.cols;
    img_ref.depth =    cvMat_ref.channels();
    img_ref.pitch =    cvMat_ref.cols*cvMat_ref.elemSize();
    img_ref.itemSize = cvMat_ref.elemSize1();
    img_ref.data =     cvMat_ref.ptr();
}

// IMU from Pixhawk
void 
Flow_Control::
_imu_callback(const GPU_2_Quad_ROS::from_px4 px4_data_i)
{
	px4data = px4_data_i;
	if (!home_set) 
	{ 
		baro_home = 0;	//px4data.baro;  velocity filter already set to 0
		yaw_home = 	px4data.yaw;
		home_set = 	true;
	}
}



// Multithreaded Loops
void
Flow_Control::
loop_flow()		// Flow Calc Loop
{	
	ROS_INFO("Flow thread started!");
	double tnow;	
	while(start)
	{
		if (img_ctr > flow_ctr)
		{
			tnow = (double)ros::Time::now().toSec();
			if ( ( (mode_use==0||mode_use==4||mode_use==5) && (mode_direct==2) ) || (mode_direct==1) )
			{
				if (flow_ctr%2==0)
				{
					flow_ctr = img_ctr;
					continue;
				}

			}
			_flow_comp();
			flow_ctr = img_ctr;
			
			/*if (flow_ctr%(int)cam_specs[0] == 0)
			{ 
				cout << "elapsed time: " << filter.elapsedTime() << " ms" << endl; 
				cout << "fulflow time: " << 1000.0*((double)ros::Time::now().toSec()-tnow) << endl;
			}*/
		}		
		usleep(100);
	}
}
void
Flow_Control:: 
loop()			// Control Calc Loop
{
	usleep(500000);
	ROS_INFO("Control thread started!");

	time_p = ros::Time::now().toSec();
	

	hover_xInt = 0;
	hover_yInt = 0;
	hover_zInt = 0;

	mode_direct = 2; // 0 for ctrl, 1 for hover, 2 for experiment

	mode_old = -1;	// start
	

	// hover integral reset
	hover_xInt = 0;
	hover_yInt = 0;
	hover_zInt = 0;	


	// First publish
	px4cmd.x_vel_des = 	0;
	px4cmd.y_vel_des = 	0;
	px4cmd.z_vel_des = 	0;
	px4cmd.vbx =		0;
	px4cmd.vby =		0;
	px4cmd.viz =		0;
	px4cmd.mode = 		-5.0f; // hover
	px4cmd.yaw_des = 	yaw_home;

	px4cmdpub.publish(px4cmd);
	ros::spinOnce();
	usleep(3000000);
	
	double time_ctrl = (double)ros::Time::now().toSec();


	while(start)
	{
		if (flow_ctr > ctrl_ctr)
		{
			time_n = ros::Time::now().toSec();
			dt = time_n - time_p;
			time_p = time_n;

			if (mode_direct == 2)
			{	
				if (mode_old<0)
				{
					mode_use = 0;	// control
				}
				if (mode_use == 0)		// hover
				{
					if (mode_use != mode_old){ printf("hovering\n"); }
	
					_hover_comp_gpu(dt);
					px4cmd.mode = 		-5.0f;
					//px4cmd.z_vel_des = 	-1.0*(px4data.baro-(baro_home-1.7f));

					if ( (double)ros::Time::now().toSec() > (time_ctrl+15.0) )
					{
						mode_use++;
						printf("hover time: %0.3f\n\n", (float)(ros::Time::now().toSec()-time_ctrl));
						time_ctrl = (double)ros::Time::now().toSec();
						yaw_home = px4data.yaw;
					}
					mode_old = 0;
				}
				else if (mode_use == 1)		// switch to forward
				{
					if (mode_use != mode_old){ printf("cam up\n"); }
					
					// Temp
					//px4data.x_vel = 1.0f; px4data.y_vel = 0.3f;

					float vi_mag =		sqrt(pow(px4data.x_vel,2)+pow(px4data.y_vel,2));
					float vi_ang =		atan2(px4data.y_vel,px4data.x_vel);
					x_vel =				vi_mag*cos(vi_ang-px4data.yaw);
					y_vel =				vi_mag*sin(vi_ang-px4data.yaw);

					px4cmd.x_vel_des = 	-0.3*x_vel;
					px4cmd.y_vel_des = 	-0.3*y_vel;
					px4cmd.z_vel_des = 	-1.0*(px4data.baro-(baro_home-1.7f));
					px4cmd.vbx =		0;
					px4cmd.vby =		0;
					px4cmd.viz =		0;
					px4cmd.mode = 		2.5f; // hover
					px4cmd.yaw_des = 	yaw_home-0.1f;

					// hover integral reset
					hover_xInt = 0;
					hover_yInt = 0;
					hover_zInt = 0;	

					if ( (double)ros::Time::now().toSec() > (time_ctrl+3.0) )
					{
						mode_use++;
						printf("cam up time: %0.3f\n\n", (float)(ros::Time::now().toSec()-time_ctrl));
						time_ctrl = (double)ros::Time::now().toSec();
					}
					mode_old = 1;
				}				
				else if (mode_use == 2)		// control dodging algoritm
				{
					if (mode_use != mode_old){ printf("ctrl\n"); }

					_vel_map();
					_costs_comp();
					_ctrl_comp(dt);

					// hover integral reset
					hover_xInt = 0;
					hover_yInt = 0;
					hover_zInt = 0;	

					/*px4cmd.x_vel_des = 	0.5f;
					px4cmd.y_vel_des = 	0;*/
					px4cmd.z_vel_des = 	-1.0*(px4data.baro-(baro_home-1.7f));

					if ( (double)ros::Time::now().toSec() > (time_ctrl+30.0) )
					{
						mode_use++;
						printf("ctrl time: %0.3f\n\n", (float)(ros::Time::now().toSec()-time_ctrl));
						time_ctrl = (double)ros::Time::now().toSec();
					}
					mode_old = 2;
				}
				else if (mode_use == 3)		// camera down
				{
					if (mode_use != mode_old){ printf("cam down\n"); }

					hatLz = log(2.0);

					// Temp
					//px4data.x_vel = 1.0f; px4data.y_vel = 0.3f;

					float vi_mag =		sqrt(pow(px4data.x_vel,2)+pow(px4data.y_vel,2));
					float vi_ang =		atan2(px4data.y_vel,px4data.x_vel);
					x_vel =				vi_mag*cos(vi_ang-px4data.yaw);
					y_vel =				vi_mag*sin(vi_ang-px4data.yaw);

					px4cmd.x_vel_des = 	-0.3*x_vel;
					px4cmd.y_vel_des = 	-0.3*y_vel;
					px4cmd.z_vel_des = 	-1.0*(px4data.baro-(baro_home-1.7f));
					px4cmd.vbx =		0;
					px4cmd.vby =		0;
					px4cmd.viz =		0;
					px4cmd.mode = 		7.5f; // hover
					px4cmd.yaw_des = 	yaw_home-0.1f;

					// hover integral reset
					hover_xInt = 0;
					hover_yInt = 0;
					hover_zInt = 0;	

					if ( (double)ros::Time::now().toSec() > (time_ctrl+3.0) )
					{
						mode_use++;
						printf("cam down time: %0.3f\n\n", (float)(ros::Time::now().toSec()-time_ctrl));
						time_ctrl = (double)ros::Time::now().toSec();
					}
					mode_old = 3;
				}
				else if (mode_use == 4)		// hover and land
				{
					if (mode_use != mode_old)
					{ 
						printf("hover end\n"); 
					}

					_hover_comp_gpu(dt);
					//px4cmd.z_vel_des = 	-1.0*(px4cmd.viz-(baro_home-1.7f));
					if ( (double)ros::Time::now().toSec() > (time_ctrl+5.0) )
					{
						px4cmd.z_vel_des = 	0.2f;


					}
					
					px4cmd.mode = 		-5.0f;

					mode_old = 4;
				}
				else if (mode_use == 5)		// hover and land
				{
					if (mode_use != mode_old)
					{ 
						printf("LANDING!\n"); 
					}

					_hover_comp_gpu(dt);
					px4cmd.z_vel_des = 	0.2f;
					px4cmd.vbx =		0.2f*px4cmd.vbx;
					px4cmd.vby =		0.2f*px4cmd.vby;
					px4cmd.x_vel_des = 	0;
					px4cmd.y_vel_des = 	0;		
					px4cmd.mode = 		-5.0f;

					mode_old = 5;
				}
			}
			else if (mode_direct == 1)
			{
				_hover_comp_gpu(dt);
				if( (double)ros::Time::now().toSec() > (time_ctrl+45.0) ) { px4cmd.z_vel_des = 0.2f; }
			}
			else if (mode_direct == 0)
			{
				_vel_map();
				_costs_comp();
				_ctrl_comp(dt);

				// hover integral reset
				hover_xInt = 0;
				hover_yInt = 0;
				hover_zInt = 0;	
			}

			ctrl_ctr = flow_ctr;
			
			//printf("time_ctrl = %0.6f\n", (float)(ros::Time::now().toSec() - time_p));
		}

		// publish and sleep

		if(px4cmd.z_vel_des>0.5f){ px4cmd.z_vel_des=0.5f; }
		if(px4cmd.z_vel_des<-0.5f){ px4cmd.z_vel_des=-0.5f; }
		px4cmd.yaw_des = 	yaw_home+0.12;
		px4cmdpub.publish(px4cmd);
		
		ros::spinOnce();
		usleep(1000);
	}
}
void
Flow_Control::
loop_display()	// Display Loop
{
	ros::Rate loop_rate(30);
	usleep(1000000);
	ROS_INFO("Display thread started!");
	disp_ctr = 0;
	double disptime;
	while(start)
	{
		if (flow_ctr > 0)
		{

			disptime = (double)ros::Time::now().toSec();
			// Get image and flow
			lockThread.lock();
			imageDCV = imageCV.clone();
			flowDispCV = flowTransCV.clone();
			ctrlCostsDCV = ctrlCostsTransCV.clone();
			lockThread.unlock();

			Mat cost_save = Mat(cam_h, cam_w, CV_8UC1);


			// Velocity mapping - for display
			if ( (vmap[1] > 0) && (vmap[1] <= cam_w) && (vmap[2] > 0) && (vmap[2] <= cam_h) )
			{
				cv::circle( imageDCV, cv::Point(vmap[1],vmap[2]), 6, 150, -1, 8 );
				cv::circle( imageDCV, cv::Point(dX,dY), 6, 255, -1, 8 );
			}

//&& (vmap[0] > 0.2f)

			// Save data
			char savedataloc[100] = "/home/nvidia/ros_ws/flowcontrol_mk1/src/data_flow/data/";
			char imgend[5] = ".jpg";
			char filenumber[20];
			sprintf(filenumber, "%d", disp_ctr);
			strcat(savedataloc, filenumber);
			strcat(savedataloc, imgend);
			float cost;

			for (int y = (0); y <= (flowDispCV.rows-1); y++)
			{
				for (int x = (0); x <= (flowDispCV.cols-1); x++)
				{
					saveDataCV.at<Vec3b>(y,x)[0] =	(uint8_t)(256/2/maxflow*flowDispCV.at<Vec2f>(y,x)[0]+256/2);
					saveDataCV.at<Vec3b>(y,x)[1] =	(uint8_t)(256/2/maxflow*flowDispCV.at<Vec2f>(y,x)[1]+256/2);
					cost =	1.0f/cmap.B*ctrlCostsDCV.at<float>(y,x);
					cost_save.at<uint8_t>(y,x) = (uint8_t)(255*cost);
				}		
			}
			imwrite(savedataloc, saveDataCV);

			// Save image
			char imagefileloc[100] = "/home/nvidia/ros_ws/flowcontrol_mk1/src/data_flow/image/";
			strcat(imagefileloc, filenumber);
			strcat(imagefileloc, imgend);
			imwrite(imagefileloc, imageDCV);

			// Save cost
			char costfileloc[100] = "/home/nvidia/ros_ws/flowcontrol_mk1/src/data_flow/cost/";
			strcat(costfileloc, filenumber);
			strcat(costfileloc, imgend);
			imwrite(costfileloc, cost_save);		


			// Display
			/*imshow("image", imageDCV);
			imshow("cost", ctrlCostsDCV);
			waitKey(3); */
			disp_ctr++;


			//printf("so it did attempt to save\n");
			loop_rate.sleep();
		}
	}
}                 

void
Flow_Control:: 
loop_wzspeed()
{
	usleep(1000000);
	ROS_INFO("wz speed thread started!");

	while(start)
	{
		if (flow_ctr > 0)
		{
			// Download flow
			lockThread.lock();
			wzflowCV = flowTransCV.clone();
			lockThread.unlock();

			wzspeed = wz_speed_comp();
		}
		usleep(1000);
	}
}


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

	flowtimefile << filter.elapsedTime() <<endl;
}















