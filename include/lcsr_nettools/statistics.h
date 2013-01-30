#ifndef __LCSR_NETTOOLS_STATISTICS_H
#define __LCSR_NETTOOLS_STATISTICS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <lcsr_nettools/TopicMeasurements.h>

namespace lcsr_nettools {

  //! ROS Topic Statistics
  /*!
   * This class measures statistics for a stream of ROS messages without
   * having to create an additional connection to the publisher.
   */

  class StatisticsTracker {
  public:
    //! Default Constructor
    StatisticsTracker(ros::NodeHandle nh,
               const std::string topic_name,
               const ros::Duration latency_buffer_duration = ros::Duration(0));

    //! Destructor
    ~StatisticsTracker();

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
    
    //! Get the msg loss
    double msg_loss(const bool all_time=false) const;
    
    //! Get the average frequenzy
    double frequency_avg(const bool all_time=false) const;
    //! Get the variance of the frequency
    double frequency_var(const bool all_time=false) const;
    //! Get the minimum frequency
    double frequency_min(const bool all_time=false) const;
    //! Get the maximum frequency
    double frequency_max(const bool all_time=false) const;

    //! Get the average latency
    double latency_avg(const bool all_time=false) const;
    //! Get the variance of the latency
    double latency_var(const bool all_time=false) const;
    //! Get the minimum latency
    double latency_min(const bool all_time=false) const;
    //! Get the maximum latency
    double latency_max(const bool all_time=false) const;

    //@}
    
    //! Get the latency of the latest package
    double latency_latest() const;
    
    //! Get the sampling window duration
    double get_window_duration() const;

    //! Fill a ROS message
    void fill_measurement_msg(lcsr_nettools::TopicMeasurements &msg, const bool all_time=false) const;

    //! Fill and publish a ROS message 
    void publish() const;

  private: 
    class MessageSample {
    public:
      uint32_t seq;
      ros::Time send_time;
      ros::Time recv_time;
      double time_sep;
    };

    static bool latency_cmp(const MessageSample& a, const MessageSample& b) {
      return (a.recv_time - a.send_time) < (b.recv_time - b.send_time);
    }

    static bool frequency_cmp(const MessageSample& a, const MessageSample& b) {
      return (1.0/a.time_sep) < (1.0/b.time_sep);
    }

    ros::NodeHandle nh_;
    std::string topic_name_;
    ros::Duration sample_buffer_duration_;

    std::list<MessageSample> samples_;
    MessageSample first_sample_;
    MessageSample latest_sample_;

    size_t n_msgs_received_;
    size_t n_msgs_out_of_order_;

    double latency_avg_;
    double latency_var_;
    double latency_min_;
    double latency_max_;

    double frequency_avg_;
    double frequency_var_;
    double frequency_min_;
    double frequency_max_;

    ros::Publisher diagnostics_pub_;
  };

}

#endif // ifndef __LCSR_NETTOOLS_STATISTICS_H
