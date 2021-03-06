#include "flowcontrol_h.hpp"

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

using namespace std;

boost::shared_ptr<Flow_Control> FCclass; 
std::thread thread_flow;
std::thread thread_ctrl;
std::thread thread_display;
std::thread thread_visservo;

typedef uint64_t UINT_PTR;

using namespace std;
using namespace mvIMPACT::acquire;
using namespace cv;

int capture_number = 0;

static bool s_boTerminated = false;


double tcap = 0;



string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}









//-----------------------------------------------------------------------------
struct CaptureParameter
//-----------------------------------------------------------------------------
{
    Device*                 pDev;

    FunctionInterface       fi;
    Statistics              statistics;
 
    CaptureParameter( Device* p ) : pDev( p ), fi( p ), statistics( p )
    {

    }
    ~CaptureParameter()
    {
    }
};



//-----------------------------------------------------------------------------
void displayImage( CaptureParameter* pCaptureParameter, Request* pRequest )
//-----------------------------------------------------------------------------
{

    //int openCVDataType = CV_8UC1;
    int openCVDataType = CV_8UC4;

	Mat image_cam, image_cam_grey;
    if (!FCclass->start)
    { 
		FCclass->start = true; 
		FCclass->lockThread.lock();
		image_cam = cv::Mat( cv::Size( pRequest->imageWidth.read(), pRequest->imageHeight.read() ), openCVDataType, pRequest->imageData.read(), pRequest->imageLinePitch.read() ); 	


		//image_cam_grey = Mat(image_cam.rows, image_cam.cols, CV_8UC1);
		//cvtColor( image_cam, image_cam_grey, CV_RGB2GRAY);
		//undistort(image_cam_grey, FCclass->imageLCV, FCclass->K, FCclass->D);

		cvtColor( image_cam, FCclass->imageLCV, CV_RGB2GRAY);
		resize(FCclass->imageLCV, FCclass->imageCV, Size(), 0.5, 0.5);

        FCclass->init(); 
        FCclass->img_ctr++;
        FCclass->lockThread.unlock();
		//printf("on init end\n");
		thread_flow = 		FCclass->spawn_flow();
		thread_display = 	FCclass->spawn_display();
		thread_ctrl = 		FCclass->spawn();
		thread_visservo = 	FCclass->spawn_visservo();
        //thread_flow.detach();
		//thread_display.detach();
		//thread_ctrl.detach();
		//thread_wzspeed.detach();

	}
    else
    { 
		FCclass->lockThread.lock();
		//double tsgat = ros::Time::now().toSec();
		image_cam = cv::Mat( cv::Size( pRequest->imageWidth.read(), pRequest->imageHeight.read() ), openCVDataType, pRequest->imageData.read(), pRequest->imageLinePitch.read() ); 	
		//printf("on else start\n");

		//image_cam_grey = Mat(image_cam.rows, image_cam.cols, CV_8UC1);
		//cvtColor( image_cam, image_cam_grey, CV_RGB2GRAY);
		//undistort(image_cam_grey, FCclass->imageLCV, FCclass->K, FCclass->D);

		cvtColor( image_cam, FCclass->imageLCV, CV_RGB2GRAY);
		resize(FCclass->imageLCV, FCclass->imageCV, Size(), 0.5, 0.5);
		//double tegat = ros::Time::now().toSec();
		//printf("distort time: %0.6f\n", tegat-tsgat);
		//printf("on else end\n");
		//if (ros::Time::now().toSec() > (tcap+1.0))
		//{
		//	printf("ctr = %d\n", FCclass->img_ctr);
		//	tcap = ros::Time::now().toSec();
		//}
        FCclass->img_ctr++;
        FCclass->lockThread.unlock();

	} 

}



//-----------------------------------------------------------------------------
unsigned int DMR_CALL liveLoop( void* pData )
//-----------------------------------------------------------------------------
{
    CaptureParameter* pThreadParameter = reinterpret_cast<CaptureParameter*>( pData );

    TDMR_ERROR result = DMR_NO_ERROR;
    while( ( result = static_cast<TDMR_ERROR>( pThreadParameter->fi.imageRequestSingle() ) ) == DMR_NO_ERROR ) {};
    if( result != DEV_NO_FREE_REQUEST_AVAILABLE )
    {
        cout << "'FunctionInterface.imageRequestSingle' returned with an unexpected result: " << result
             << "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
    }

    manuallyStartAcquisitionIfNeeded( pThreadParameter->pDev, pThreadParameter->fi );
    
	// run thread loop
    unsigned int cnt = 0;
    const unsigned int timeout_ms = -1;
    Request* pRequest = 0;
    Request* pPreviousRequest = 0;
    while( !s_boTerminated )
    {
        // wait for results from the default capture queue
        int requestNr = pThreadParameter->fi.imageRequestWaitFor( timeout_ms );
        pRequest = pThreadParameter->fi.isRequestNrValid( requestNr ) ? pThreadParameter->fi.getRequest( requestNr ) : 0;
        if( pRequest )
        {
            if( pRequest->isOK() )
            {
                ++cnt;
                // here we can display some statistical information every 100th image
                /*if( cnt % 60 == 0 )
                {
                    cout << "Info from " << pThreadParameter->pDev->serial.read()
                         << ": " << pThreadParameter->statistics.framesPerSecond.name() << ": " << pThreadParameter->statistics.framesPerSecond.readS()
                         << ", " << pThreadParameter->statistics.errorCount.name() << ": " << pThreadParameter->statistics.errorCount.readS()
                         << ", " << pThreadParameter->statistics.captureTime_s.name() << ": " << pThreadParameter->statistics.captureTime_s.readS()
                         << ", CaptureDimension: " << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << "(" << pRequest->imagePixelFormat.readS() << ")" << endl;
                }*/

				
                displayImage( pThreadParameter, pRequest );

            }
            else
            {
                cout << "Error: " << pRequest->requestResult.readS() << endl;
            }
            if( pPreviousRequest )
            {
                // this image has been displayed thus the buffer is no longer needed...
                pPreviousRequest->unlock();
            }
            pPreviousRequest = pRequest;
            // send a new image request into the capture queue
            pThreadParameter->fi.imageRequestSingle();
        }
        else
        {
            cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ")"
                 << ", timeout value too small?" << endl;
        }

		// End if keyboard input
        s_boTerminated = checkKeyboardInput() == 0 ? false : true;
    }

    manuallyStopAcquisitionIfNeeded( pThreadParameter->pDev, pThreadParameter->fi );
    cv::destroyAllWindows();

    return 0;
}



//-----------------------------------------------------------------------------
void runLiveLoop( CaptureParameter& captureParams )
//-----------------------------------------------------------------------------
{
    s_boTerminated = false;
    liveLoop( &captureParams );
}



//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
//-----------------------------------------------------------------------------
{
    DeviceManager devMgr;
    Device* pDev = getDeviceFromUserInput( devMgr );

	ros::init(argc, argv, "flowcontrol");
	FCclass.reset(new Flow_Control());
	ros::NodeHandle nm;

	// Check device opened
    if( !pDev )
    { cout << "Unable to continue!"; return 0; }

    cout << "Initialising the device. This might take some time..." << endl;

    try { pDev->open(); }
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "An error occurred while opening device " << pDev->serial.read()
             << "(error code: " << e.getErrorCodeAsString() << ")." << endl;
    }
    CaptureParameter captureParams( pDev );
    string settingName = "/home/nvidia/ros_ws/hover_wflow/src/mvSettings.xml";	 ///home/nvidia/ros_ws/flowcontrol_mk1/src/mvSettings.xml  
    if( !settingName.empty() )
    {
        cout << "Trying to load setting " << settingName << "..." << endl;
        int result = captureParams.fi.loadSetting( settingName );
        if( result != DMR_NO_ERROR )
        {
            cout << "loadSetting( \"" << settingName << "\" ); call failed: " << ImpactAcquireException::getErrorCodeAsString( result ) << endl;
        }
    }
	
    //=============================================================================
    //========= Capture loop into memory managed by the driver (default) ==========
    //=============================================================================
    cout << "The device will try to capture continuously into memory automatically allocated be the device driver." << endl
         << "This is the default behaviour." << endl;
	cout << "Press [ENTER] to end the application..." << endl;
    runLiveLoop( captureParams );
 
    // and end the application
	cout << "terminating (main end) " << endl;
	FCclass->lockThread.lock();
	FCclass->start = false; 
	FCclass->lockThread.unlock();
	thread_visservo.join();
	thread_display.join();
	thread_flow.join();
	thread_ctrl.join();
	return 0;
}
