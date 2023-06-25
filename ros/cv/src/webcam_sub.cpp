#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "FaceDetector.h"
#include <std_msgs/Char.h>

void faceCenter(cv::Rect face, cv::Mat frame)
{
  ros::NodeHandle nh2;

  ros::Publisher pub = nh2.advertise<std_msgs::Char>(
      "/camera_dir", 10);
  std_msgs::Char msg;

  int x = face.x + face.width / 2;
  int y = face.y + face.height / 2;
  int w = frame.size().width;
  int h = frame.size().height;
  int x_center = w / 2;
  int y_center = h / 2;
  if (abs(x - x_center) > 100)
  {
    if (x < x_center)
    {
      std::cout << "left" << std::endl;
      msg.data = char('l');
    }
    else
    {
      std::cout << "right" << std::endl;
      msg.data = char('r');
    }
  }
  else
  {
    std::cout << "center" << std::endl;
    msg.data = char('c');
  }

  // if (y < y_center)
  // {
  //   std::cout << "up" << std::endl;
 
  // }
  // else
  // {
  //   std::cout << "down" << std::endl;

  // }

  // Publish the message
  pub.publish(msg);
  ros::spinOnce();
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

  // Pointer used for the conversion from a ROS message to
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;

  FaceDetector face_detector;
  try
  {

    // Convert the ROS message
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    cv::Mat current_frame = cv_ptr->image;

    // Detect faces in the current frame
    auto rectangles = face_detector.detect_face_rectangles(current_frame);
    cv::Scalar color(0, 105, 205);
    for (const auto &r : rectangles)
    {
      // std::cout << r.x << ' \n';
      cv::rectangle(current_frame, r, color, 4);
    }
    if (rectangles.size() > 0)
    {
      faceCenter(rectangles[0], current_frame);
    }

    // Display the current frame
    cv::imshow("view", current_frame);

    // Display frame for 30 milliseconds
    cv::waitKey(10);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "frame_listener");

  // Default handler for nodes in ROS
  ros::NodeHandle nh;

  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);

  // Subscribe to the /camera topic
  image_transport::Subscriber sub = it.subscribe("usb_cam_node/image_raw", 1, imageCallback);

  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();

  // Close down OpenCV
  cv::destroyWindow("view");
}