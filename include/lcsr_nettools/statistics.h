#ifndef __LCSR_NETTOOLS_STATISTICS_H
#define __LCSR_NETTOOLS_STATISTICS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace lcsr_nettools {

  //! ROS Topic Statistics
  /*!
   * This class measures statistics for a stream of ROS messages without
   * having to create an additional connection to the publisher.
   */

  class Statistics {
  public:
    //! Default Constructor
    Statistics(const std::string &topic_name,
               const ros::Duration &latency_buffer_duration = ros::Duration(0));

    //! Destructor
    ~Statistics();

    // @name Sempling Functions
    // Description of group 2.
    //@{
    
    //! Sample a message header
    void sample(const std_msgs::Header &header,
                const ros::Time &time = ros::Time::now());

    //@}

    // @name Measurement Functions
    // Description of group 2.
    //@{
    
    //! Get the packet loss
    double packet_loss(const bool recent=false) const;
    //! Get the minimum latency
    ros::Duration latency_min(const bool recent=false) const;
    //! Get the maximum latency
    ros::Duration latency_max(const bool recent=false) const;
    //! Get the average latency
    ros::Duration latency_average(const bool recent=false) const;
    //! Get the variance of the latency
    ros::Duration latency_variance(const bool recent=false) const;

    //! Get the latency of the latest package
    ros::Duration latency_latest() const;
    
    //@}

  private: 
    class MessageSample {
    public:
      uint32_t seq;
      ros::Time send_time;
      ros::Time recv_time;
    };

    static bool latency_cmp(const MessageSample& a, const MessageSample& b) {
      return (a.recv_time - a.send_time) < (a.recv_time - a.send_time);
    }

    std::string topic_name_;
    ros::Duration latency_buffer_duration_;

    std::list<MessageSample> latencies_;
    MessageSample first_sample_;
    MessageSample latest_sample_;

    size_t n_msgs_received_;
    size_t n_msgs_out_of_order_;

    ros::Duration total_latency_;
    ros::Duration total_latency_squares_;
    ros::Duration min_latency_;
    ros::Duration max_latency_;
  };

}

#endif // ifndef __LCSR_NETTOOLS_STATISTICS_H
