
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <lcsr_nettools/statistics.h>
#include <lcsr_nettools/HeaderSample.h>

void sample_header(const lcsr_nettools::HeaderSample::ConstPtr &header_sample,
                   lcsr_nettools::StatisticsTracker &stat_tracker,
                   bool throttle,
                   double throttle_factor)
{
  ROS_DEBUG("Received sample.");
  stat_tracker.sample(header_sample->header, header_sample->recv_time);
  stat_tracker.publish(throttle, throttle_factor);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "statistics_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Get topic name
  std::string topic_name = "undefined";
  nh_private.param("topic_name",topic_name,topic_name);

  // Get sample buffer duration
  double sample_buffer_duration = 30.0;
  nh_private.param("sample_buffer_duration",
                   sample_buffer_duration,
                   sample_buffer_duration);

  // Get throttle flag
  bool throttle = true;
  nh_private.param("throttle",throttle,throttle);
  double throttle_factor = 0.5;
  nh_private.param("throttle_factor",
                   throttle_factor,
                   throttle_factor);

  // Create StatisticsTracker
  lcsr_nettools::StatisticsTracker stat_tracker(nh, topic_name, ros::Duration(sample_buffer_duration));
  stat_tracker.enable(true);

  // Subscribe to the header samples
  ros::Subscriber header_sample_sub = 
    nh.subscribe<lcsr_nettools::HeaderSample>("header_samples",
                                              10,
                                              boost::bind(sample_header, _1, stat_tracker, throttle, throttle_factor));

  ros::spin();

  return 0;
}
