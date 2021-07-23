#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_publisher");
  ros::NodeHandle nh;

  // ros parameters
  std::string device_id;
  ros::param::param<std::string>("~device", device_id, "0");

  std::string device_xr;
  ros::param::param<std::string>("~devxr", device_xr, "640");

  std::string device_yr;
  ros::param::param<std::string>("~devyr", device_yr, "480");

  int view;
  ros::param::param<int>("~view", view, 1);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  // build the video capture parameters
  std::string capture_args = "";
  capture_args.append("nvarguscamerasrc sensor-id=");
  capture_args.append(device_id);
  capture_args.append(" ! video/x-raw(memory:NVMM), width=");
  capture_args.append(device_xr);
  capture_args.append(", height=");
  capture_args.append(device_yr);
  capture_args.append(", format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=");
  capture_args.append(device_xr);
  capture_args.append(", height=");
  capture_args.append(device_yr);
  capture_args.append(", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink");

  // open cam
  VideoCapture cam(capture_args, cv::CAP_GSTREAMER);
  if(!cam.isOpened())
  {
      printf("Camera is not opened.\n");
      return -1;
  }

  ros::Rate loop_rate(30);
  while (nh.ok()) {

    // read frame
    Mat frame;
    cam >> frame;

    if(view)
    {
      // show frame
      imshow("original",frame);

      // exit on esc
      if((char)waitKey(30) == 27)
          break;
    }

    if(!frame.empty()){
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
