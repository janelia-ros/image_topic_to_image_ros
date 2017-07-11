#include <ros/ros.h>
#include <nodelet/loader.h>
#include <image_stream_to_heatmap/advertisement_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_stream_to_heatmap");

  // Check for common user errors
  if (ros::names::remap("camera") != "camera")
  {
    ROS_WARN("Remapping 'camera' has no effect! Start image_stream_to_heatmap in the "
             "camera namespace instead.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=%s rosrun image_stream_to_heatmap image_stream_to_heatmap",
             ros::names::remap("camera").c_str());
  }
  if (ros::this_node::getNamespace() == "/")
  {
    ROS_WARN("Started in the global namespace! This is probably wrong. Start image_stream_to_heatmap "
             "in the camera namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_camera rosrun image_stream_to_heatmap image_stream_to_heatmap");
  }

  // Shared parameters to be propagated to nodelet private namespaces
  ros::NodeHandle private_nh("~");
  XmlRpc::XmlRpcValue shared_params;
  int queue_size;
  if (private_nh.getParam("queue_size", queue_size))
    shared_params["queue_size"] = queue_size;

  nodelet::Loader manager(false); // Don't bring up the manager ROS API
  nodelet::M_string remappings(ros::names::getRemappings());
  nodelet::V_string my_argv;

  std::string create_file_name = ros::this_node::getName() + "_create_file";
  remappings["camera/image_raw"] = ros::names::resolve("image_raw");
  remappings["camera/camera_info"] = ros::names::resolve("camera_info");
  if (shared_params.valid())
    ros::param::set(create_file_name, shared_params);
  manager.load(create_file_name, "image_stream_to_heatmap/create_file", remappings, my_argv);

  // Check for only the original camera topics
  ros::V_string topics;
  topics.push_back(ros::names::resolve("image_raw"));
  topics.push_back(ros::names::resolve("camera_info"));
  image_stream_to_heatmap::AdvertisementChecker check_inputs(ros::NodeHandle(), ros::this_node::getName());
  check_inputs.start(topics, 60.0);

  ros::spin();
  return 0;
}
