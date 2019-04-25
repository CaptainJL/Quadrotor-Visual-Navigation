
#include "flowcontrol_h.hpp"


// Relevant Namespaces used
using namespace std;
using namespace cv;
using namespace flowfilter;
using namespace flowfilter::gpu;
using namespace Eigen;




// Velocity Mapping   
void
Flow_Control::
_vel_map()
{
	// Determine 
	float yaw = 		px4data.yaw; // Roll and Pitch is 0 for gyro stabilised camera
	float x_vel_i = 	px4data.x_vel;
	float y_vel_i = 	px4data.y_vel;

	// Convert inertial to body-direction (aka, 0 roll and pitch)
	float vi_mag =		sqrt(pow(x_vel_i,2)+pow(y_vel_i,2));
	float vi_ang =		atan2(y_vel_i,x_vel_i);

	x_vel =				vi_mag*cos(vi_ang-yaw);
	y_vel =				vi_mag*sin(vi_ang-yaw);


	// TESTING ONLY
	/*x_vel = 			0.8f;
	y_vel = 			0;
	px4data.y_vel =		y_vel;
	px4data.x_vel =		x_vel;*/


	float v_xy_ang = 	atan2(y_vel,x_vel);
	float v_xy_mag = 	sqrt(pow(x_vel,2)+pow(y_vel,2));
	//float v_z_ang = 	atan2(px4data.z_vel, v_xy_mag);
	float v_z_ang = 	0;
	
  	// Difference in alt and azi
  	float alt_diff = 	v_z_ang;
  	float azi_diff = 	-v_xy_ang; //+yaw // yaw = 0 for body-fixed mapping

  	// Velocity map to Camera
  	float x_map = 	-cam_specs[1]*tan(azi_diff) + cam_specs[3];
  	float y_map = 	cam_specs[2]*tan(alt_diff) + cam_specs[4];

  	if ( (x_map > 0) && (x_map <= cam_w) && (y_map > 0) && (y_map <= cam_h))
  	{
		vmap[1] =		x_map;
		vmap[2] = 		y_map;
		vmap[0]	=		v_xy_mag;
  	}


}
// Costs Comp   
void
Flow_Control::
_costs_comp()
{
	// Download flow an upload to GPU
	lockThread.lock();
	flowCtrlCV = flowTransCV.clone();
	lockThread.unlock();

	float2 flow;
	int2 pix;
	float cost;

	// Calculate Costs
	for (int y=0; y<cam_h; y++)
	{
		for (int x=0; x<cam_w; x++)
		{
			pix = {x,y};
			flow.x = flowCtrlCV.at<Vec2f>(y,x)[0];
			flow.y = flowCtrlCV.at<Vec2f>(y,x)[1];
			cost = _cost_px(pix, flow);
			ctrlCostsCV.at<float>(y,x) = cost;
		}
	}
	lockThread.lock();
	ctrlCostsTransCV = ctrlCostsCV.clone();
	lockThread.unlock();


}
// Cost per pixel
float
Flow_Control::
_cost_px(int2 pix, float2 flow)
{
	float r = 0.4f, zref_gain = 1.5f;
	gains[0] = 1; gains[1] = 1;

  	// Magnitude references
  	float flow_mag = sqrt(pow(flow.x,2)+pow(flow.y,2));
	if (sqrt(pow(pix.x-vmap[1],2)+pow(pix.y-vmap[2],2)) < 40)
	{ flow_mag = 0; }


  	float u_bar = 	pow(pix.x-vmap[1],2) + pow(pix.y-vmap[2],2);
	float zref = 	zref_gain*vmap[0];

    float Zr;
    if (sqrt(u_bar)<5)
    {
    	Zr =	r*cam_specs[1]/1.0f;
    }
    else
    {
		Zr =	r*cam_specs[1]/sqrt(u_bar);
    }

    float mu_r =  	zref/2;
    float d = zref;
    // float d = (r*cam_charsgpu[1])/sqrt(u2_bar);
    float mu;
    if (Zr<mu_r) 
    {
    	mu = 1;
    }
    else if (Zr<d)
    {
    	mu = (d-Zr)/(d-mu_r);
    }
    else
    {
    	mu = 0;
    }

	if(mu>1.0f){ mu=1.0f; }



  	// Sigma_X
  	float sigma_x = cam_specs[0] * flow_mag/vmap[0] * (r*cam_specs[1])/(u_bar) * mu;
  	// Sigma_Y
  	
  	//float sigma_y = cam_specs[0] * flow_mag/vmap[0] * zref/sqrt(u_bar);

	float gamma_x;
	if (sigma_x>1)
	{
		gamma_x = 1;
	}
	else
	{
		gamma_x= cmap.k*( expf(cmap.alpha*sigma_x)-1.0f );	
	}

	float gamma = gamma_x;
    if (sqrt(u_bar) < 25)
    { gamma = 0; }

	if (vmap[0]<0.25){ gamma = 0; }
	if (gamma>cmap.B){ gamma = cmap.B; }

	return gamma;
}



// Ctrl Comp   
void
Flow_Control::
_ctrl_comp(double dt)
{
	// Init vals
	float sum_l=0, sum_r=0, sum_t=0, sum_b=0;
	float cnt_l=0, cnt_r=0, cnt_t=0, cnt_b=0;
	float cost;

	// Calculate gamma sums (left, right, top, bottom)
	for (int y=0; y<cam_h; y++)
	{
		for (int x=0; x<cam_w; x++)
		{		
			cost = ctrlCostsCV.at<float>(y,x);
			// If infinite (-1 denotes infinite)
			if (cost==-1){cost=5000;}

			// Compare and calc gamma sums
			if (x<vmap[1]){ sum_l+=cost; cnt_l++; }
			if (x>vmap[1]){ sum_r+=cost; cnt_r++; }
			if (y<vmap[2]){ sum_t+=cost; cnt_t++; }
			if (y>vmap[2]){ sum_b+=cost; cnt_b++; }	
		}
	}
	// Take care if counts are 0 (would create div 0 error)
	cnt_l = (cnt_l==0) ? 1 : cnt_l;
	cnt_r = (cnt_r==0) ? 1 : cnt_r;
	cnt_t = (cnt_t==0) ? 1 : cnt_t;
	cnt_b = (cnt_b==0) ? 1 : cnt_b;

	// Calc averages
	float avg_l = sum_l/cnt_l;
	float avg_r = sum_r/cnt_r;
	float avg_t = sum_t/cnt_t;
	float avg_b = sum_b/cnt_b;

	// Average totals
	//float avg_hori = avg_r-avg_l;	// Left side is positive yaw
	//float avg_vert = avg_b-avg_t;	// Top is negative z direction



	// Average totals and limiting
	float xd = 		avg_r-avg_l;	// Left side is positive yaw
	float yd = 		avg_b-avg_t;	// Top is negative z direction
	xd = -xd;
	float diffMax = 25;
	if(xd>diffMax){xd=diffMax;}
	if(xd<-diffMax){xd=-diffMax;}
	if(yd>diffMax){yd=diffMax;}
	if(yd<-diffMax){yd=-diffMax;}

	// SPEED CONTROL
	double wz =		wzspeed;
	double wz_ref =	3.0f;
	dvwz =	-1.0*(vwz - wz_ref + wz);		
	vwz += 	dvwz;	
	if(vwz>1.7){ vwz = 1.0; }	
	if(vwz<0.4){ vwz = 0.4; }

	vwz = 1.25f;

 



	// Control Y velocity
  	double p = 1, z = 1000;
	double ke = 20;

	double ep_d = p*ke*(xd-xd_p)/z/dt;
	double ep_p = p*ke*xd_p;
	double epsilon = epsilon_p - dt*(p*epsilon_p + ep_d + ep_p); //
	if (epsilon>0.4){epsilon=0.4;}
	if (epsilon<-0.4){epsilon=-0.4;}
	epsilon_p = epsilon;
	xd_p = xd;
	
	double yv_cmd = y_vel - epsilon;



	//printf("wz, vwz:  %0.3f, %0.3f\n", wz, vwz);

	// Epsilon conditioning
	if ( yv_cmd>(0.5*vwz) ) 
	{ 
		yv_cmd = (0.5*vwz)-0.2f;
		epsilon_p = 0.2f;	 
	}
	if ( yv_cmd<(-0.5*vwz) ) 
	{ 
		yv_cmd = (-0.5*vwz)+0.2f;
		epsilon_p = -0.2f;	 
	}


	// Control Z Velocity
	float zv_cmd = 	-1.0*(px4data.baro-(baro_home-1.7f));
	float xv_cmd = (float)vwz;

	// If too low velocity:
	if (vmap[0]<0.25 || x_vel<0 || abs(y_vel)>abs(x_vel))
	{
		xv_cmd = 0.75f;
		yv_cmd = 0;
		epsilon = 0;
		epsilon_p = 0;
		// sigma = 0;
		// sigma_p = 0;

	}




	// Mapping
	float alt_diff = 0;
	float azi_diff = -atan2(yv_cmd,xv_cmd);;

	dX = -cam_specs[1]*tan(azi_diff) + cam_specs[3];
	dY = cam_specs[2]*tan(alt_diff) + cam_specs[4];



	px4cmd.x_vel_des = 	(float)xv_cmd;
	px4cmd.y_vel_des = 	(float)yv_cmd;
	px4cmd.z_vel_des = 	zv_cmd;
	if(px4cmd.z_vel_des>0.3f){ px4cmd.z_vel_des=0.3f; }
	if(px4cmd.z_vel_des<-0.3f){ px4cmd.z_vel_des=-0.3f; }
	px4cmd.mode =		20.0f; // ctrl mode
	px4cmd.yaw_des = 	yaw_home;


	

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
	float wx = 			0.3f/1000.0f*sumflowY/(float)wf_gpu.wf_params.L0;
	float wy = 			-0.3f/1000.0f*sumflowX/(float)wf_gpu.wf_params.L1;
	float wz = 			-1.6f/1000.0f*sumflowZ/(float)wf_gpu.wf_params.L2;

	// Position Integrals
	if(abs(wx)>0.1f){ hover_xInt += (float)dt*wx; }
	if(abs(wy)>0.1f){ hover_yInt += (float)dt*wy; }
	if(abs(wz)>0.1f){ hover_zInt += (float)dt*wz; }


	// Calculate controller
	//float xy_pgain = 0.5f;
	float z_pgain = 1.0f;

	//float ctrl_vbx = -xy_pgain*hover_xInt;
	//float ctrl_vby = -xy_pgain*hover_yInt; 
	//float ctrl_viz = -z_pgain*(hover_zInt - -2.0f); // 2m hold

	
	//  hatLz Filter
	double dhatLz = -wz;
	hatLz +=		dt*dhatLz ;
	double z_wz =	-exp(hatLz); 

	if (mode_use==4 && abs(wz)>0.4f)
	{ mode_use = 5; }


	// use baro alt
	float ctrl_viz = -z_pgain*(px4data.baro-(baro_home-1.7f));
	float ctrl_vbx = 0;
	float ctrl_vby = 0;

	if(mode_use<0)
	{
		ctrl_viz = -z_pgain*(z_wz-(baro_home-1.7f));	// In lab
	}


	// Land after 40s
	/*if ((double)ros::Time::now().toSec() > (start_time+40.0))
	{
		ctrl_vbx = -wx*0.2f;
		ctrl_vby = -wy*0.2f;
		ctrl_viz = 0.2f;
	}*/

	ctrl_viz = -z_pgain*(z_wz-(baro_home-1.7f));

	// Publish
	px4cmd.x_vel_des = 	ctrl_vbx;
	px4cmd.y_vel_des = 	ctrl_vby;
	px4cmd.z_vel_des = 	ctrl_viz;
	px4cmd.vbx =		wx;
	px4cmd.vby =		wy;
	px4cmd.viz =		z_wz;
	px4cmd.mode = 		-5.0f; // hover
	px4cmd.yaw_des = 	yaw_home;

	//printf("xpos, ypos: %0.3f, %0.3f\n", hover_xInt, hover_yInt);
	//printf("wflow: %0.3f, %0.3f, %0.3f\n", wx, wy,wz);

}









double
Flow_Control::
wz_speed_comp()
{
	
	double f = 		(double)cam_specs[1];	
	double freq =	(double)cam_specs[0];
	double rad = 	170.0;

	double theta_0 = 	atan2(rad,f);
	//double lambda =		(sin(theta_0)*sin(theta_0)) / (4.0-sin(theta_0)*sin(theta_0));
	double Lambda_r =	PI*pow(sin(theta_0),4)/4.0;

	double L2 =		Lambda_r*2.0;
	double S_cap = 	2.0*PI*(1.0-cos(theta_0));

	double flowX, flowY, xd, yd, xymag, theta, S_area;

	double x0 = (double)vmap[1];
	double y0 = (double)vmap[2];

	// Structure Flow
  	double sflow_z = 0, sflow_zsum = 0;

	for (int y = (0); y <= (wzflowCV.rows-1); y++)
	{
		for (int x = (0); x <= (wzflowCV.cols-1); x++)
		{
			flowX = 		(double)wzflowCV.at<Vec2f>(y,x)[0];
			flowY = 		(double)wzflowCV.at<Vec2f>(y,x)[1];
			// Simplifiers
			xd =			x-x0;
			yd =			y-y0;
			xymag = 		sqrt( xd*xd + yd*yd );

			theta =	atan2(xymag,f);
			if( theta<theta_0 && xymag>5.0 )
			{
				S_area = pow(cos(theta),3)/(f*f);

				sflow_z = -S_area/S_cap*f/xymag*tan(theta)*(flowX*xd/xymag + flowY*yd/xymag);
				sflow_zsum += sflow_z;
			}
			else
			{
				sflow_zsum += 0;
			}
		}
	}

	double wz = abs(freq*L2*sflow_zsum);

	return wz;

}




















/* ORIGINAL
// Cost per pixel
float
Flow_Control::
_cost_px(int2 pix, float2 flow)
{
	float r = 0.3, zref_gain = 1;
	gains[0] = 1; gains[1] = 1;

  	// Magnitude references
  	float flow_mag = sqrt(pow(flow.x,2)+pow(flow.y,2));
	if (sqrt(pow(pix.x-vmap[1],2)+pow(pix.y-vmap[2],2)) < 40)
	{ flow_mag = 0; }
	// Test 
	//vmap[0]=0.5;

  	float u_bar = pow(pix.x-vmap[1],2) + pow(pix.y-vmap[2],2);

  	// Sigma_X
  	float sigma_x = cam_specs[0] * flow_mag/vmap[0] * (r*cam_specs[1])/(u_bar);
  	// Sigma_Y
  	float zref = zref_gain*vmap[0];
  	float sigma_y = cam_specs[0] * flow_mag/vmap[0] * (pow(cam_specs[1],2)*zref)/sqrt(u_bar);

  	// NOTE: infinite values defined as -1
  	// Gamma_X, Gamma_Y
  	float gamma_x, gamma_y;
	gamma_x = (abs(sigma_x)>0.95) ? -1 : (gains[0]*tan(3.1415f/2.0f * abs(sigma_x)));
	gamma_y = (abs(sigma_y)>0.95) ? -1 : (gains[0]*tan(3.1415f/2.0f * abs(sigma_y)));

	// Testing of control
	//if (flow_mag>0.1){gamma_x=0.75; gamma_y=0.75;}
	//if (pix.x<vmap[1]){gamma_x=0.75; gamma_y=0.75;}
	//else {gamma_x=0; gamma_y=0;}

	//float rp;
	// sigma_y sel
	//if (vmap[0]!=0)
	//{ rp = 	cam_specs[1]*r/vmap[0]; }
	//else
	//{ rp = cam_specs[1]*r; }

	//float gamma;
	//if ( sqrt(u_bar) < rp )
	//{ gamma = gamma_y; }
	//else
	//{ gamma = gamma_x; }

  	
	// Gamma ORIGINAL 
	float gamma;
	if (gamma_x>=0 && gamma_y>=0)
  	{ gamma = gamma_x*gamma_y; }
  	else if (gamma_x>=0 && gamma_y<0)
  	{ gamma = gamma_x; }
  	else if (gamma_x<0 && gamma_y>=0)
  	{ gamma = gamma_y; }
  	else
  	{ gamma = -1; } 

	if (vmap[0]<0.25){ gamma = 0; }

	if (gamma<0.2f){ gamma = 0; }


	return gamma;
}*/



























