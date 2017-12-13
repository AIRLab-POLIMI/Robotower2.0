/* Get images frames from available camera and publish frames using image_transport data type.
	Author: Ewerton Lopes
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("ext_usb_camera/image", 1);

  // Convert the passed as command line parameter index for the video device to an integer
  bool setIndex = true;
  int source_index = 0;


  cv::VideoCapture cap(source_index);
  ROS_INFO_STREAM("Camera device ready! Index value: " << source_index);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(30);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      std_msgs::Header time_stamp;
      time_stamp.stamp = ros::Time::now();
      msg = cv_bridge::CvImage(time_stamp, "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
