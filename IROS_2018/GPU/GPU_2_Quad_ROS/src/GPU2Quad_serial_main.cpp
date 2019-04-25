#include "GPU2Quad_serial.h"

// Pre-declarations
void open_port(void);
void configure_port(void);
void write_port(void);
void read_port(void);
float interpret_data(uint8_t);
double time_diff(struct timeval, struct timeval);
void sigint(int);

uint8_t buffer_s[40] = {0};
uint8_t buffer_r[52] = {0};
uint8_t rBufTemp[52] = {0};



using namespace std;

// ROS
ros::Publisher to_GPU;
GPU_2_Quad_ROS::from_px4 to_GPUd;
ros::Subscriber pub_2PX4;

union i8_f2 
{
	float f;
	uint32_t i;
} v;

void callback(const GPU_2_Quad_ROS::to_px4 px4_to)
{
	x_vel_des = px4_to.x_vel_des;
	y_vel_des = px4_to.y_vel_des;
	z_vel_des = px4_to.z_vel_des;
	flow_vbx = px4_to.vbx;
	flow_vby = px4_to.vby;
	flow_viz = px4_to.viz;
	mode_sel = 	px4_to.mode;
	yaw_des = 	px4_to.yaw_des;
}


// OPEN PORT AND CHECK
void open_port(void)
{
	 // file description for the serial port
	//close(3);
	printf("open port\n");
	port_id = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY );
	printf("open port: completed\n");

	if(port_id < 0) // if open is unsucessful
	{
		printf("Error: Unable to open port '%s'. \n", port_name);
		port_status = false;
		exit(1);		
	}
	else
	{
		fcntl(port_id, F_SETFL, 0);
		printf("Port '%s' (id: %d) is open.\n", port_name, port_id);
		configure_port();
	}
} //open_port

void configure_port(void)      // configure the port
{
	printf("Port is configuring...\n");
	
	//memset(&config, 0, sizeof(config));

	tcgetattr(port_id, &config);

	int bs_in = 	cfsetispeed(&config, BAUD_RATE);
	int bs_out =    cfsetospeed(&config, BAUD_RATE);
	if ((bs_in < 0) || (bs_out < 0))
	{
		fprintf(stderr, "\nERROR: Could not set desired baud rate of Baud\n");
		close(port_id);
		exit(1);
	}

	config.c_cflag     &=  ~PARENB;        // Make 8n1
	config.c_cflag     &=  ~CSTOPB;
	config.c_cflag     &=  ~CSIZE;
	config.c_cflag     |=  CS8;
	config.c_cflag     &=  ~CRTSCTS;       // no flow control
	config.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
	config.c_oflag     =   0;                  // no remapping, no delays
	config.c_cc[VMIN]      =   0;                  // read doesn't block
	config.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

	config.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
	config.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
	config.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	config.c_oflag     &=  ~OPOST;              // make raw
	



	
	if ( tcsetattr(port_id, TCSANOW, &config) >= 0 )
	{
	    // apply the settings to the port
	    printf("Port configured and ready\n\n");
	    port_status = true;
	}
	else
	{
		printf("Failed to configure\n\n");
		close(port_id);
		exit(1);
	}

} //configure_port






// WRITE DATA TO PORT
void write_port(void)
{
	//x_vel_des = 0.9f;
	//y_vel_des = -1.4f;
	//z_vel_des = 0.3f;
	// aye
	//mode_sel = -5.0f;
	// Covert to u_int32_t type
	uint32_t x_vel_des_h = 	*(uint32_t *)&x_vel_des;
	uint32_t y_vel_des_h = 	*(uint32_t *)&y_vel_des;
	uint32_t z_vel_des_h = 	*(uint32_t *)&z_vel_des;
	uint32_t flow_vbx_h = 	*(uint32_t *)&flow_vbx;
	uint32_t flow_vby_h = 	*(uint32_t *)&flow_vby;
	uint32_t flow_viz_h = 	*(uint32_t *)&flow_viz;
	uint32_t mode_sel_h =	*(uint32_t *)&mode_sel;
	uint32_t yaw_des_h =	*(uint32_t *)&yaw_des;
	float avg_h =			(x_vel_des+y_vel_des+z_vel_des + flow_vbx+flow_vby+flow_viz + mode_sel+yaw_des)/8.0f;
	uint32_t avg_o = 		*(uint32_t *)&avg_h;

	// Assign buffer values
	uint8_t i;
	for (i = 0; i<4; i++)
	{

		buffer_s[i] = 		( serial_header	>> 8*(3-i) ) & 0xff;
		buffer_s[i+4] = 	( x_vel_des_h	>> 8*(3-i) ) & 0xff;
		buffer_s[i+8] = 	( y_vel_des_h 	>> 8*(3-i) ) & 0xff;
		buffer_s[i+12] = 	( z_vel_des_h 	>> 8*(3-i) ) & 0xff;
		buffer_s[i+16] = 	( flow_vbx_h 	>> 8*(3-i) ) & 0xff;
		buffer_s[i+20] = 	( flow_vby_h 	>> 8*(3-i) ) & 0xff;
		buffer_s[i+24] = 	( flow_viz_h 	>> 8*(3-i) ) & 0xff;
		buffer_s[i+28] = 	( mode_sel_h 	>> 8*(3-i) ) & 0xff;
		buffer_s[i+32] = 	( yaw_des_h 	>> 8*(3-i) ) & 0xff;
		buffer_s[i+36] = 	( avg_o 		>> 8*(3-i) ) & 0xff;
	}

	/*printf("data output: \n");
	for (int i=0; i<sizeof(buffer_s); i++)
	{
		printf("%02x",buffer_s[i]);
	}
	printf("\n");*/

	// Write data to serial port
	write(port_id, buffer_s, sizeof(buffer_s));
	memset(buffer_s,0,sizeof(buffer_s));

}


void read_port(void)
{
	uint8_t vh = 0;
	int td = 0;

	int pcount = 9*sizeof(int);

	while(1)
	{
		usleep(1);
		int rvh = read(port_id, &vh, 1);
		if (vh == 0x48)
		{ 	
		while(1)
			{
				ioctl(port_id, FIONREAD, &td);
				if (td >= (pcount-1))
				{
					buffer_r[0] = vh;
					read(port_id, &rBufTemp, (pcount-1));
	
					vh = 0;
					for (uint8_t rc = 1; rc < pcount; rc++)
					{
						buffer_r[rc] = rBufTemp[rc-1];
					}
					break;
					
				}
			}
		}
		if (td >= (pcount-1)) { break; }
	}

	/*for (uint8_t i = 0; i < pcount; i++)
	{
		printf("%02x",buffer_r[i]);
	}
	printf("\n");*/


	sh = 			interpret_data(0);
	roll = 			interpret_data(1);
	pitch = 		interpret_data(2);
	yaw = 			interpret_data(3);
	x_vel = 		interpret_data(4);
	y_vel = 		interpret_data(5);
	z_vel = 		interpret_data(6);
	barometer = 	interpret_data(7);
	avg = 			interpret_data(8);
	avg_calc = 		(roll+pitch+yaw + x_vel+y_vel+z_vel + barometer)/7.0f;
	//printf("avg,avg_c = %0.2f %0.2f\n", avg, avg_calc);

	if ( (*(uint32_t *)&sh == serial_header) && ((avg_calc+0.01f) > avg) && ((avg_calc-0.01f) < avg) )
	{
		to_GPUd.roll = 			roll;
		to_GPUd.pitch = 		pitch;
		to_GPUd.yaw = 			yaw;
		to_GPUd.x_vel = 		x_vel;
		to_GPUd.y_vel = 		y_vel;
		to_GPUd.z_vel = 		z_vel;
		to_GPUd.baro =			barometer;
		to_GPU.publish(to_GPUd);
		//printf("Correct Data!\n");
	}
	else
	{
		//printf("incorrect data!\n");
	}

	memset(&buffer_r, 0, 48);
	memset(&rBufTemp, 0, 48);
	//printf("\n");
}

// Convert bytes to floats
float interpret_data(uint8_t i)
{
	
	//i8_f2 v;
	//uint32_t v;
	
	v.i = 		0;
	v.i =  		buffer_r[0+4*i];
	v.i <<= 	8;
	v.i |= 		buffer_r[1+4*i];
	v.i <<= 	8;
	v.i |= 		buffer_r[2+4*i];
	v.i <<= 	8;
	v.i |= 		buffer_r[3+4*i];
	
	return v.f;
}








// MAIN
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Tegra_PX4");
	ros::NodeHandle n;
	ros::Rate freq(TELEM_FREQ);
	to_GPU = n.advertise<GPU_2_Quad_ROS::from_px4>("px4_from", 100);
	pub_2PX4 = n.subscribe<GPU_2_Quad_ROS::to_px4>("px4_to", 100, callback);

	signal(SIGINT, sigint);
	open_port();
	gettimeofday(&before , NULL);
	double b4;
	
	while(port_status && ros::ok())
	{ 	
		// b4 = ros::Time::now().toSec();
			
		read_port();
		usleep(5000);
		write_port();

		ros::spinOnce();
		// printf("Freq: %0.6f\n", 1.0/((double)ros::Time::now().toSec()-b4));
		
	}
}

// TIME
double time_diff(struct timeval x , struct timeval y)
{
    double x_ms , y_ms , diff;
     
    x_ms = (double)x.tv_sec*1000000 + (double)x.tv_usec;
    y_ms = (double)y.tv_sec*1000000 + (double)y.tv_usec;
     
    diff = (double)y_ms - (double)x_ms;
     
    return diff;
}

void sigint(int a)
{
	printf("Shutting down!\n\n");
	close(port_id);
	port_status = false;
	exit(0);
}



