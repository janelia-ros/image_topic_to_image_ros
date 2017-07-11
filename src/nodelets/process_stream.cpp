#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <ros/console.h>


namespace image_stream_to_heatmap {

class ProcessStreamNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_,it_out_;
  image_transport::Subscriber image_sub_;
  int queue_size_;
  cv_bridge::CvImageConstPtr source_ptr_;

  boost::mutex callback_mutex_;
  image_transport::Publisher image_pub_;

  cv::Mat image_sum_;
  bool image_sum_initialized_;

  virtual void onInit();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

};

void ProcessStreamNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_out(nh,"heatmap");
  it_in_.reset(new image_transport::ImageTransport(nh));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // read parameters
  private_nh.param("queue_size",queue_size_,5);

  image_sum_initialized_ = false;

  image_pub_ = it_out_->advertise("image_raw",1);
  image_sub_ = it_in_->subscribe("/camera/blob_out/image_raw",queue_size_,&ProcessStreamNodelet::imageCb,this);
}

void ProcessStreamNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
{
  // get a cv::Mat view of the source data
  {
    boost::lock_guard<boost::mutex> lock(callback_mutex_);
    source_ptr_ = cv_bridge::toCvShare(image_msg,sensor_msgs::image_encodings::BGR8);
  }

  // input image
  cv::Mat image_input;
  cv::cvtColor(source_ptr_->image,image_input,CV_BGR2GRAY);

  if (image_sum_initialized_)
  {
    cv::add(image_sum_,image_input,image_sum_,cv::noArray(),CV_64F);
  }
  else
  {
    image_sum_ = image_input;
    image_sum_.convertTo(image_sum_,CV_64F);
    image_sum_initialized_ = true;
  }

  cv::Mat image_sum_normalized;
  normalize(image_sum_,image_sum_normalized,0,255,cv::NORM_MINMAX,CV_8U);

  // output image
  cv::Mat image_output;
  applyColorMap(image_sum_normalized,image_output,cv::COLORMAP_JET);

  sensor_msgs::ImagePtr image_msg_ptr = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_output).toImageMsg();
  image_pub_.publish(image_msg_ptr);

}

} // namespace image_stream_to_heatmap

// register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( image_stream_to_heatmap::ProcessStreamNodelet,nodelet::Nodelet)
