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
	
	// Distortion params
	K = (Mat_<float>(3,3) << 214.218f, 0, 362.525f, 0, 214.266f, 230.211f, 0, 0, 1.0f); //0.2434f
	D = (Mat_<float>(1,5) << -0.12345f, 0.01313f, 0, 0, -0.0006f);

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
	cam_specs[0] = 	60;		// 45
	cam_specs[1] = 	180;	// 240
	cam_specs[2] = 	180;
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
	wf_gpu.wf_params.rad = 		120.0;

	double theta_0 = 	atan2(wf_gpu.wf_params.rad,wf_gpu.wf_params.f);
	double lambda =		(sin(theta_0)*sin(theta_0)) / (4.0-sin(theta_0)*sin(theta_0));
	double Lambda_r =	PI*pow(sin(theta_0),4)/4.0;

	wf_gpu.wf_params.L0 =		Lambda_r/lambda;
	wf_gpu.wf_params.L1 =		Lambda_r/lambda;
	wf_gpu.wf_params.L2 =		Lambda_r*2.0;
	wf_gpu.wf_params.S_cap = 	2.0*PI*(1.0-cos(theta_0));


	// hatLz filter
	hatLz = log(0.1);
	aruco_height = 0;

	// Visual servoing
	visservo_xcmd = 0;
	visservo_ycmd = 0;
	visservo_sx = 	-1;
	visservo_sy =	-1;
	vs_x_des = 0;
	vs_y_des = 0;



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
			//printf("t_flow: %0.6f\n", (double)ros::Time::now().toSec()-tnow);	
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

	vs_y_des = 0;
	vs_x_des = 0;


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
	double time_now = (double)ros::Time::now().toSec();
	double time_ref = 0;

	//bool tap = true;


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
				//px4cmd.x_vel_des = 	0;
				//px4cmd.y_vel_des = 	0;
				//px4cmd.z_vel_des = 	-0.5f;
				px4cmd.z_vel_des = 	-0.7f*(-aruco_height-(-2.0));
				
				px4cmd.mode = 		-5.0f;
				px4cmd.yaw_des = 	yaw_home;

				if ( (double)ros::Time::now().toSec() > (time_ctrl+10.0) )
				{
					mode_direct++;
					time_ctrl = (double)ros::Time::now().toSec();
					printf("Hold Baro!\n");
				}
			}			
			else if (mode_direct==2)		// Slow to zero z vel
			{
				//px4cmd.x_vel_des = 	0;
				//px4cmd.y_vel_des = 	0;
				px4cmd.z_vel_des = 	-0.7f*(-aruco_height-(-2.0));;
				px4cmd.mode = 		-5.0f;
				px4cmd.yaw_des = 	yaw_home;

				if ( (double)ros::Time::now().toSec() > (time_ctrl+5.0) )
				{
					mode_direct++;
					time_ctrl = (double)ros::Time::now().toSec();
					printf("Hold!\n");
					vs_x_des = 0.0f;
					vs_y_des = 0.0f;
				}
			}	
			else if (mode_direct==3)		// Manveuver for 35s
			{
				//px4cmd.x_vel_des = 	0;
				//px4cmd.y_vel_des = 	0;
				//px4cmd.z_vel_des = 	0;
				px4cmd.mode = 		-5.0f;
				px4cmd.yaw_des = 	yaw_home;
				
				time_now = (double)ros::Time::now().toSec();
				time_ref = time_now - time_ctrl;



				
				// Hover + shift pos
				if ( time_now>=(time_ctrl+10.0) && time_now<(time_ctrl+15.0) )
				{
					vs_y_des = 1.5f;
				}
				else if ( time_now>=(time_ctrl+15.0) && time_now<(time_ctrl+20.0) )
				{
					vs_y_des = 0.0f;
				}
				else if ( time_now>=(time_ctrl+25.0) && time_now<(time_ctrl+30.0) )
				{
					vs_y_des = -1.5f;
				}
				else
				{
					vs_y_des = 0.0f;
				}


				// Hover + shift pos
				/*if ( time_now>=(time_ctrl+10.0) && time_now<(time_ctrl+15.0) )
				{
					vs_x_des = 0.0f;
					vs_y_des = 1.0f;
				}
				else if ( time_now>=(time_ctrl+15.0) && time_now<(time_ctrl+20.0) )
				{
					vs_x_des = 0.5f;
					vs_y_des = 0.0f;
				}
				else if ( time_now>=(time_ctrl+20.0) && time_now<(time_ctrl+25.0) )
				{
					vs_x_des = 0.0f;
					vs_y_des = -1.0f;
				}
				else if ( time_now>=(time_ctrl+25.0) && time_now<(time_ctrl+30.0) )
				{
					vs_x_des = -0.5f;
					vs_y_des = 0.0f;
				}
				else
				{
					vs_x_des = 0.0f;
					vs_y_des = 0.0f;
				}*/




				/* // Fig8
				if ( (time_ref>=10.0) && (time_ref<70.0) )
				{
					vs_x_des = 0.5f*sin(2.0f*PI/(2.0f*15.0f)*(time_ref-10.0f));
					vs_y_des = 0.8f*cos(2.0f*PI/15.0f*(time_ref-10.0f));
				}
				else
				{
					vs_x_des = 0.0f;
				}*/

				if ( time_now > (time_ctrl+40.0) )
				{
					mode_direct++;
					time_ctrl = (double)ros::Time::now().toSec();
					printf("Landing!\n");
				}
			}	
			else if (mode_direct==4)		// Land
			{
				//px4cmd.x_vel_des = 	0;
				//px4cmd.y_vel_des = 	0;
				px4cmd.z_vel_des = 	0.35;

				px4cmd.mode = 		-5.0f;
				px4cmd.yaw_des = 	yaw_home;
			}	


			ctrl_ctr = flow_ctr;
			
			//printf("time_ctrl = %0.6f\n", (float)(ros::Time::now().toSec() - time_p));
		}

		// publish and sleep

		if(px4cmd.z_vel_des>0.5f){ px4cmd.z_vel_des=0.5f; }
		if(px4cmd.z_vel_des<-0.5f){ px4cmd.z_vel_des=-0.5f; }
		px4cmd.yaw_des = 	yaw_home; //+0.25f; //+0.15f
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
			imageDCV = imageLCV.clone();		// grey
			flowDispCV = flowTransCV.clone();
			lockThread.unlock();
			
			// Save data
			char savedataloc[100] = "/home/nvidia/ros_ws/hover_wflow/src/data_flow/data/";
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


			// Set circle
			if (visservo_sx>0 && visservo_sy>0)
			{
				circle( imageDCV, Point( visservo_sx, visservo_sy ), 8.0, 128, -1);
			}
			// Save image
			char imagefileloc[100] = "/home/nvidia/ros_ws/hover_wflow/src/data_flow/image/";
			strcat(imagefileloc, filenumber);
			strcat(imagefileloc, imgend);
			imwrite(imagefileloc, imageDCV);

		


			// Display
			//imshow("image", imageDCV);
			//imshow("blue seg", segDCV);
			//waitKey(10); 
			disp_ctr++;

			//if (disp_ctr%60 == 0)
			//{
			//    printf("saving time = %0.6f\n", (double)ros::Time::now().toSec()-disptime);
			//}

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
	double tnow,tcpy;
	ROS_INFO("Visual Servoing thread started!");
	ros::Rate vis_rate(100);

	float Kdata[9] = { 130, 0, 376, 0, 130, 240, 0, 0, 1 };
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, Kdata);
	cv::Mat distCoeffs; 
	cv::Ptr<cv::aruco::Dictionary> dictionary =   cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	while(start)
	{
		if (img_ctr > vis_ctr)
		{
			tnow = (double)ros::Time::now().toSec();
		
			lockThread.lock();
			imageUC2CV = imageLCV.clone();
			lockThread.unlock();
			//undistort(imageUC2CV, imageC2CV, K, D);
			imageC2CV = imageUC2CV.clone();
			tcpy = (double)ros::Time::now().toSec();



			/* CHESSBOARD 
			int cbx = 5, cby = 4;
			Size patternsize(cbx,cby);
			vector<Point2f> corners;

			bool cb_detected = findChessboardCorners( imageC2CV, patternsize, corners,  CALIB_CB_NORMALIZE_IMAGE ); //  | CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK
			//printf("detected? %d\n", cb_detected);

			int xpos = 0, ypos = 0;

			int g_x, g_y;

			if (cb_detected)
			{
				for (int i = 0; i<(cbx*cby); i++)
				{
					xpos += corners[i].x;
					ypos += corners[i].y;
				}
				float gfx =  (int)xpos/(int)(cbx*cby);
				float gfy =  (int)ypos/(int)(cbx*cby);
				g_x = (int)gfx;
				g_y = (int)gfy;
				//circle( imageC2CV, Point( g_x, g_y ), 8.0, 255, -1);
			}
			else
			{
				g_x = -1;
				g_y = -1;

			} */
 


			/* ARUCO MARKERS */
			int xpos = 0, ypos = 0;
			int g_x, g_y;
			std::vector<std::vector<cv::Point2f> > corners; 
			std::vector<int> ids; 
			cv::aruco::detectMarkers(imageC2CV, dictionary, corners, ids);

			if (ids.size() > 0)
			{	
	
				vector< Vec3d > rvecs, tvecs;
				cv::aruco::estimatePoseSingleMarkers(corners, 0.02, cameraMatrix, distCoeffs, rvecs, tvecs);
				std::vector<cv::Point2f> crnrs = corners[0];
				Vec3d aruco_pos = tvecs[0];	
				aruco_height = 35.0*aruco_pos[2]; // Needs a ridiculously high gain for some reason, but is very stable				
				float cx = 1.0f/4.0f*(crnrs[0].x + crnrs[1].x + crnrs[2].x + crnrs[3].x);
				float cy = 1.0f/4.0f*(crnrs[0].y + crnrs[1].y + crnrs[2].y + crnrs[3].y);
				g_x = (int)cx;
				g_y = (int)cy;
				//circle( imageC2CV, Point( g_x, g_y ), 8.0, 255, -1);
				//printf("aruco detected! g_x g_y = %d %d, xyz = %0.3f %0.3f %0.3f\n", g_x, g_y, aruco_pos[0], aruco_pos[1], aruco_height);
				//printf("aruco detected! z= %0.3f\n", aruco_height);
			}
			else
			{
				g_x = -1;
				g_y = -1;
				//printf("aruco detect failed!\n");
			} 






			// Control vx, vy
			float kpos_visservo = 	0.2f;
			float assumed_height = 	2.0f;	// Expected height off ground
			float f = 				180.0f; //376.0f; //235.0f; //200.0f; 
			float x0 = 				752.0f/2.0f;
			float y0 =				480.0f/2.0f;

			//if (mode_direct==3)
			//{			
			// 	assumed_height =	exp(hatLz);
			//}

			float quad_x_pos =		assumed_height*(g_y-y0)/f;	// x pos is along camera -y
			float quad_y_pos =		-assumed_height*(g_x-x0)/f;  // y pos is along camera x
			
			// printf("quad pos x,y: %0.3fx %0.3fyf\n", quad_x_pos, quad_y_pos);

			if (mode_direct!=3)
			{
				kpos_visservo = 0.2f;
			}

			float ctrl_x =			-kpos_visservo*(quad_x_pos-vs_x_des);
			float ctrl_y =			-kpos_visservo*(quad_y_pos-vs_y_des);
			if (g_x<0 && g_y<0)
			{
				ctrl_x = 0;
				ctrl_y = 0;
				
			}
			else
			{
				hover_xInt = 	quad_x_pos;	
				hover_yInt =	quad_y_pos;
			}


			// Publish data to system
			//lockThread.lock();
			visservo_xcmd = ctrl_x;
			visservo_ycmd = ctrl_y;
			visservo_sx = 	g_x;
			visservo_sy = 	g_y;

			//lockThread.unlock();

			//if (vis_ctr%60==0)
			//{
				//printf("timetaken: cpy, since cpy: %0.6f %0.6f\n", tcpy-tnow, (double)ros::Time::now().toSec()-tcpy);
			//}	
			
			//if (visservo_sx>0){ cout << "detected vsx" << endl; }
			//if (g_x>0) { cout << "detected g_x" << endl; }

			vis_ctr = img_ctr;	
		}		
		//usleep(1000);
		vis_rate.sleep();
	}
}

















