/* ROS related includes */
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/* OpenCV related includes */
#include "opencv2/opencv.hpp"
/* ... */

#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "robogame_cam_recorder");
    ros::NodeHandle nh;
    
    cout << "OpenCV version : " << CV_VERSION << endl;
    
	image_transport::ImageTransport it(nh);
    image_transport::Publisher imgPub = it.advertise("camera/ext_image", 1);
	
	// open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    cv::VideoCapture cap(0);
     
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    
    if(!cap.isOpened())
        cout << "failed!" << endl;
        return 0;
    
    cout << "Cap openned!" << endl;
    
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    cout << "Resolution set!" << endl;
        
    // ROS loop at 30Hz until the node is shut down.
    ros::Rate rate(30);

    //! [loop start] -- DATA AQUISITION
    while(ros::ok())
    {
        cv::Mat frame;
        bool bSuccess = cap.read(frame); // read a new frame from video
        
        sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
        imgPub.publish(imgmsg);


        if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
        
        /* Publishes the image. NOTE:: Here, the rgb image  is in fact a bgr image (bgra8, i.e., CV8U4). see http://wiki.ros.org/   cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages */
        
       
		// waitKey needed for showing the plots with cv::imshow
		char key = cv::waitKey(10);
		if(key == 27)
 			break;
 			
 		//Wait until it's time for another iteration.
        //rate.sleep();
    }
    
    // the camera will be closed automatically upon exit
    return 0;
}
