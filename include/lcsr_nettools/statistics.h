#ifndef __LCSR_NETTOOLS_STATISTICS_H
#define __LCSR_NETTOOLS_STATISTICS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <lcsr_nettools/HeaderSample.h>
#include <lcsr_nettools/TopicMeasurements.h>

namespace lcsr_nettools {

  //! ROS Topic Statistics
  /*!
   * This class measures statistics for a stream of ROS messages without
   * having to create an additional connection to the publisher.
   */

  class StatisticsTracker {
  public:
    StatisticsTracker(ros::NodeHandle nh,
               const std::string topic_name,
               const ros::Duration latency_buffer_duration = ros::Duration(0));

    ~StatisticsTracker();

    /** @name Sampling Functions
     * These functions are used to sample data.
     * @{ */
    
    //! Sample a message header
    void sample(const std_msgs::Header &header,
                const ros::Time &recv_time = ros::Time::now());

    /* @} */

    /** @name Measurement Functions
     * All of these functions can either be called to
     * process just the messages in the buffer (default, \c
     * all_time=false), or for all data collected since the
     * tracker has been reset (\c all_time=true).
     * @{ */
    
    //! Get the msg loss
    double msg_loss(const bool all_time=false) const;
    
    //! Get the average frequency
    double frequency_avg(const bool all_time=false) const;
    //! Get the standard deviation of the frequency
    double frequency_std(const bool all_time=false) const;
    //! Get the minimum frequency
    double frequency_min(const bool all_time=false) const;
    //! Get the maximum frequency
    double frequency_max(const bool all_time=false) const;

    //! Get the average latency
    double latency_avg(const bool all_time=false) const;
    //! Get the standard deviation of the latency
    double latency_std(const bool all_time=false) const;
    //! Get the minimum latency
    double latency_min(const bool all_time=false) const;
    //! Get the maximum latency
    double latency_max(const bool all_time=false) const;

    /* @} */
    
    //! Get the latency of the latest package
    double latency_latest() const;

    //! Enable or disable tracking statistics
    void enable(const bool enable);
    //! Check if statistics are being tracked / published
    bool is_enabled() const;

    //! Reset all statistics
    void reset();
    
    //! Set the topic being monitored
    void set_topic(const std::string topic_name);
    //! Get the topic being monitored
    std::string get_topic() const;
    
    //! Set the sampling window duration
    void set_window_duration(const ros::Duration sample_buffer_duration);
    //! Get the sampling window duration
    double get_window_duration() const;

    /** Fill and publish a ROS message 
     *
     * \arg \c throttle If \c throttle is set to \c true, then no
     * matter how often this function is called, it will only
     * publish at a period of \ref sample_buffer_duration_ * \c
     * throttle_factor.
     *
     * \arg \c throttle_factor The factor by which the throttle
     * period is scaled.
     */
    void publish(bool throttle=true, double throttle_factor=0.5) const;

  protected: 
    
    //! Fill a ROS message
    void fill_measurement_msg(lcsr_nettools::TopicMeasurements &msg, const bool all_time=false) const;

  private:

    class MessageSample {
    public:
      //! The squence id of this sample.
      uint32_t seq;
      //! The time this sample was sent.
      ros::Time send_time;
      //! The time this sample was received.
      ros::Time recv_time;
      //! The time between the reception of the previous sample
      //and this sample.
      ros::Duration time_sep;
    };

    //! Comparator for MessageSample latencies.
    static bool latency_cmp(const MessageSample& a, const MessageSample& b) {
      return (a.recv_time - a.send_time) < (b.recv_time - b.send_time);
    }

    //! Comparator for MessageSample frequencies.
    static bool frequency_cmp(const MessageSample& a, const MessageSample& b) {
      return (1.0/a.time_sep.toSec()) < (1.0/b.time_sep.toSec());
    }

    ros::NodeHandle nh_;
    bool enabled_;
    std::string topic_name_;
    ros::Duration sample_buffer_duration_;

    std::list<MessageSample> samples_;
    MessageSample first_sample_;
    MessageSample latest_sample_;

    size_t n_msgs_received_;
    size_t n_msgs_out_of_order_;

    lcsr_nettools::StatisticsMeasurements latency_;
    lcsr_nettools::StatisticsMeasurements frequency_;

    ros::Publisher diagnostics_pub_;
  };

}

#endif // ifndef __LCSR_NETTOOLS_STATISTICS_H
