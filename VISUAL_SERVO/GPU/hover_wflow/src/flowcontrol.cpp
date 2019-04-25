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
	vis_ctr =	0;
	start = 0;

	home_set = false;

	px4refsub = node.subscribe<GPU_2_Quad_ROS::from_px4>("px4_from", 10, &Flow_Control::_imu_callback, this);
	px4cmdpub = node.advertise<GPU_2_Quad_ROS::to_px4>("px4_to", 10);

	start_time = (double)ros::Time::now().toSec();
	
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
	cam_specs[0] = 	60;
	cam_specs[1] = 	235;	// 240
	cam_specs[2] = 	235;
	cam_specs[3] = 	imageCV.rows/2;
	cam_specs[4] = 	imageCV.cols/2;
  	cam_h = 		imageCV.rows;
  	cam_w = 		imageCV.cols;
	
  	// Image Structures
  	flowCV =		Mat(cam_h, cam_w, CV_32FC2);	_wrapCVMat(flowCV, flow);
 	flowCtrlCV = 	Mat(cam_h, cam_w, CV_32FC2);	_wrapCVMat(flowCtrlCV, flowCtrl);
  	flowDispCV = 	Mat(cam_h, cam_w, CV_32FC2);	
	// hoverRefCV = 	Mat(cam_h, cam_w, CV_32FC3);	
	// transport
	flowTransCV = 	Mat(cam_h, cam_w, CV_32FC2);
	// save data
	saveDataCV =	Mat(cam_h, cam_w, CV_8UC3);
	// Hover
	sflowCV =		Mat(cam_h, cam_w, CV_32FC3); 	_wrapCVMat(sflowCV, sflow);
	wf_gpu.sflow_gpu.upload(sflow);
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

	// Visual servoing
	visservo_xcmd = 0;
	visservo_ycmd = 0;
	visservo_sx = 	-1;
	visservo_sy =	-1;

}

// Image Wrapper
void
Flow_Control::
_wrapCVMat(Mat& cvMat_ref, image_t& img_ref) 
{	
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
			// if (flow_ctr%2==0)
			// {
			// 	flow_ctr = img_ctr;
			// 	continue;
			// }
			
			_flow_comp();
			flow_ctr = img_ctr;
			
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

	mode_direct = 0;	

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

			_hover_comp_gpu(dt);

			if (mode_direct==0)		// Wait 10 seconds before takeoff
			{
				px4cmd.x_vel_des = 	0;
				px4cmd.y_vel_des = 	0;
				px4cmd.z_vel_des = 	0;
				//px4cmd.vbx =		0;
				//px4cmd.vby =		0;
				px4cmd.viz =		0;
				px4cmd.mode = 		-100.0f;	// Minimised hold mode 
				px4cmd.yaw_des = 	yaw_home;

				if ( (double)ros::Time::now().toSec() > (time_ctrl+10.0) )
				{
					mode_direct++;
					time_ctrl = (double)ros::Time::now().toSec();
					//yaw_home = px4data.yaw;
					printf("Takeoff!\n");
				}
			}
			else if (mode_direct==1)		// Go up approx 1.5m (via 0.5m over 3s)
			{
				px4cmd.x_vel_des = 	0;
				px4cmd.y_vel_des = 	0;
				px4cmd.z_vel_des = 	-0.5;
				px4cmd.mode = 		-5.0f; 
				px4cmd.yaw_des = 	yaw_home;

				if ( (double)ros::Time::now().toSec() > (time_ctrl+2.5) )
				{
					mode_direct++;
					time_ctrl = (double)ros::Time::now().toSec();
					printf("Hold!\n");
				}
			}			
			else if (mode_direct==2)		// Hold for 30s
			{
				px4cmd.x_vel_des = 	0;
				px4cmd.y_vel_des = 	0;
				//px4cmd.z_vel_des = 	0;
				px4cmd.mode = 		-5.0f; 
				px4cmd.yaw_des = 	yaw_home;

				if (visservo_sx>0 && visservo_sy>0)
				{
					px4cmd.x_vel_des = 	visservo_xcmd;
					px4cmd.y_vel_des = 	visservo_ycmd;
				}

				if ( (double)ros::Time::now().toSec() > (time_ctrl+30.0) )
				{
					mode_direct++;
					time_ctrl = (double)ros::Time::now().toSec();
					printf("Landing!\n");
				}
			}	
			else if (mode_direct==3)		// Land
			{
				px4cmd.x_vel_des = 	0;
				px4cmd.y_vel_des = 	0;
				px4cmd.z_vel_des = 	0.35;

				if ( (double)ros::Time::now().toSec() < (time_ctrl+2.0) )
				{
					if (visservo_sx>0 && visservo_sy>0)
					{
						px4cmd.x_vel_des = 	visservo_xcmd;
						px4cmd.y_vel_des = 	visservo_ycmd;
					}
				}


				px4cmd.mode = 		-5.0f; 
				px4cmd.yaw_des = 	yaw_home;
			}	


			ctrl_ctr = flow_ctr;
			
			//printf("time_ctrl = %0.6f\n", (float)(ros::Time::now().toSec() - time_p));
		}

		// publish and sleep

		if(px4cmd.z_vel_des>0.5f){ px4cmd.z_vel_des=0.5f; }
		if(px4cmd.z_vel_des<-0.5f){ px4cmd.z_vel_des=-0.5f; }
		px4cmd.yaw_des = 	yaw_home+0.15f;
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
	usleep(1500000);
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
			imageDCV = imageCCV.clone();	// imageCV for gray, imageCCV for colour
			flowDispCV = flowTransCV.clone();
			segDCV = segTransCV.clone();
			lockThread.unlock();

			// Save data
			char savedataloc[100] = "/media/captainjl/DATA/ros_ws/hover_wflow/src/data_flow/data/"; ///home/nvidia/ros_ws/flowcontrol_mk1/src/data_flow/data/
			char imgend[5] = ".jpg";
			char filenumber[20];
			sprintf(filenumber, "%d", disp_ctr);
			strcat(savedataloc, filenumber);
			strcat(savedataloc, imgend);

			for (int y = (0); y <= (flowDispCV.rows-1); y++)
			{
				for (int x = (0); x <= (flowDispCV.cols-1); x++)
				{
					saveDataCV.at<Vec3b>(y,x)[0] =	(uint8_t)(256/2/maxflow*flowDispCV.at<Vec2f>(y,x)[0]+256/2);
					saveDataCV.at<Vec3b>(y,x)[1] =	(uint8_t)(256/2/maxflow*flowDispCV.at<Vec2f>(y,x)[1]+256/2);
					saveDataCV.at<Vec3b>(y,x)[2] = 	0;
				}		
			}
			imwrite(savedataloc, saveDataCV);

			// Save image
			char imagefileloc[100] = "/media/captainjl/DATA/ros_ws/hover_wflow/src/data_flow/image/"; ///home/nvidia/ros_ws/flowcontrol_mk1/src/data_flow/image/
			strcat(imagefileloc, filenumber);
			strcat(imagefileloc, imgend);
			imwrite(imagefileloc, imageDCV);

			// Set circle
			if (visservo_sx>0 && visservo_sy>0)
			{
				circle( segDCV, Point( visservo_sx, visservo_sy ), 8.0, 128, -1);
			}


			// Save segmentation of blue
			char segfileloc[100] = "/media/captainjl/DATA/ros_ws/hover_wflow/src/data_flow/segmented/"; ///home/nvidia/ros_ws/flowcontrol_mk1/src/data_flow/image/
			strcat(segfileloc, filenumber);
			strcat(segfileloc, imgend);
			imwrite(segfileloc, segDCV);

			// Display
			// imshow("image", imageDCV);
			// imshow("blue seg", segDCV);
			// waitKey(10); 
			disp_ctr++;

			// if (disp_ctr%60 == 0)
			// {
			// 	printf("saving time = %0.6f\n", (double)ros::Time::now().toSec()-disptime);
			// }

			//printf("so it did attempt to save\n");
			loop_rate.sleep();
		}
	}
}                  


void
Flow_Control::
loop_visservo()	// Visual Servoing Loop
{
	usleep(500000);
	double tnow;
	ROS_INFO("Visual Servoing thread started!");

	while(start)
	{
		if (img_ctr > vis_ctr)
		{
			tnow = (double)ros::Time::now().toSec();
		
			lockThread.lock();
			imageC2CV = imageCCV.clone();
			lockThread.unlock();

			// hsv convertion
			Mat hsv;
			cvtColor(imageC2CV, hsv, CV_RGB2HSV);  	//HSV 
			vector<Mat> image_hsv;
			split(hsv,image_hsv);

			uint8_t h, s;

			Mat blue_output = image_hsv[2].clone();

			// Dedicate colour spectrum
			for (int y=0; y<imageC2CV.rows; y++)
			{
				for (int x=0; x<imageC2CV.cols; x++)
				{
					h = (uint8_t)image_hsv[0].at<char>(y,x);
					s = (uint8_t)image_hsv[1].at<char>(y,x);

					if (h>5 && h<40 && s>120 && s<180)  //(h>45 && h<60 && s>70 && s<90 
					{
						blue_output.at<char>(y,x) = 255;
					}
					else
					{
						blue_output.at<char>(y,x) = 0;
					}
					// if (x==360 && y==120)
					// {
					// 	printf("h,s: %d %d\n", h, s);
					// 	printf("h,s: %d %d char\n",image_hsv[0].at<char>(y,x), image_hsv[2].at<char>(y,x));
					// }


				}	
			}

			// Median filter to elimate noise 
			medianBlur(blue_output, blue_output, 9);


			// Center position
			int cntg = 0;
			double gcs = 0, grs = 0;

			for (int y=0; y<imageC2CV.rows; y++)
			{
				for (int x=0; x<imageC2CV.cols; x++)
				{
					if (blue_output.at<char>(y,x) != 0)
					{
						gcs += (double)x;
						grs += (double)y;
						cntg++;				
					}
				}
			}

			int g_x, g_y;
			if (cntg > 10)
			{
				g_x = (int)( gcs/cntg );
				g_y = (int)( grs/cntg );
			}
			else
			{
				g_x = -1;
				g_y = -1;	
			}




			// Control vx, vy
			float kpos_visservo = 	1.0f;
			float assumed_height = 	1.5f;	// Expected height off ground
			float f = 				2.0f*235;
			float x0 = 				752.0f/2.0f;
			float y0 =				480.0f/2.0f;

			float quad_x_pos =		assumed_height*(g_y-y0)/f;	// x pos is along camera -y
			float quad_y_pos =		-assumed_height*(g_x-x0)/f;  // y pos is along camera x
			// printf("quad pos x,y: %0.3fx %0.3fyf\n", quad_x_pos, quad_y_pos);


			float ctrl_x =			-kpos_visservo*(quad_x_pos-0);
			float ctrl_y =			-kpos_visservo*(quad_y_pos-0);


			circle( blue_output, Point( g_x, g_y ), 8.0, 128, -1);

			// imshow("H", image_hsv[0]);
			// imshow("S", image_hsv[1]);
			// imshow("Green", blue_output);
			// waitKey(10);


			// Publish data to system
			lockThread.lock();
			segTransCV = blue_output.clone();
			visservo_xcmd = ctrl_x;
			visservo_ycmd = ctrl_y;
			visservo_sx = 	g_x;
			visservo_sy = 	g_y;
			lockThread.unlock();

			if (vis_ctr%60==0)
			{
				printf("timetaken: %06f\n", (double)ros::Time::now().toSec()-tnow);
			}	
			
			vis_ctr = img_ctr;	
		}		
		usleep(1000);
	}
}

















