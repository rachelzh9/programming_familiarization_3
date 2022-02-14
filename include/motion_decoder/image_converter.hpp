#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    P = = {656.4037475585938, 0.0, 329.7264464396903, 0.0, 0.0, 655.7791748046875, 261.0592521875151, 0.0, 0.0, 0.0, 1.0, 0.0};
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void setTagLocations(float x_det, float y_det, float z_det)
  {
	  //TODO: Update tag locations
    float w = P[8]*x_det + P[9]*y_det + P[10]*z_det + P[11];
    x_loc = (P[0]*x_det + P[1]*y_det + P[2]*z_det + P[3])/w;
    y_loc = (P[4]*x_det + P[5]*y_det + P[6]*z_det + P[7])/w;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	//TODO: Draw circles at tag locations on image. 
    cv::circle(cv_ptr->image, cv::Point(x_loc,y_loc), 20, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // TODO:Output modified video stream
    // Convert the modified frames into sensor_msgs::Image message and publish it using image_pub
    image_pub_.publish(cv_ptr->toImageMsg());
  }

private:
  float x_loc ,y_loc;
  double P[12];
  std::vector<float> x_arr;
  std::vector<float> y_arr;
};
