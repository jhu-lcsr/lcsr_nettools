
#include <lcsr_nettools/statistics.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <boost/foreach.hpp>
#include <lcsr_nettools/TopicStatistics.h>
#include <lcsr_nettools/TopicMeasurements.h>
#include <lcsr_nettools/StatisticsMeasurements.h>

using namespace lcsr_nettools;

StatisticsTracker::StatisticsTracker(ros::NodeHandle nh,
                       const std::string topic_name,
                       const ros::Duration sample_buffer_duration) :
  nh_(nh),
  enabled_(false),
  topic_name_(topic_name),
  sample_buffer_duration_(sample_buffer_duration),
  samples_(),
  first_sample_(),
  n_msgs_received_(0),
  n_msgs_out_of_order_(0),
  latency_(),
  frequency_(),
  diagnostics_pub_(nh_.advertise<lcsr_nettools::TopicStatistics>("topic_statistics",10))
{

}

StatisticsTracker::~StatisticsTracker() {

}

void StatisticsTracker::sample(const std_msgs::Header &header,
                               const ros::Time &recv_time)
{
  if(!enabled_) {
    return;
  }

  // Increment the sample counter
  n_msgs_received_++;
  
  // Check if this message is out of order
  if(header.seq < samples_.back().seq) {
    n_msgs_out_of_order_++;
  }

  // Compute the latency
  double latency = (recv_time - header.stamp).toSec();
  double latency_avg_last = latency_.avg;
  // Compute running total latency, min latency, max latency
  latency_.avg += (latency-latency_.avg)/double(n_msgs_received_);
  latency_.std = std::sqrt(((n_msgs_received_ - 1) * std::pow(latency_.std,2)
                            + (latency - latency_.avg) * (latency - latency_avg_last)) / n_msgs_received_);
  latency_.min = (latency < latency_.min)?(latency):(latency_.min);
  latency_.max = (latency > latency_.max)?(latency):(latency_.max);

  // Compute the frequency of this 
  double frequency = (n_msgs_received_ < 2) ? (0.0) : (1.0 / (recv_time - latest_sample_.recv_time).toSec());
  double frequency_avg_last = frequency_.avg;
  // Compute running average frequency, min frequency, max frquency
  frequency_.avg += (frequency-frequency_.avg)/double(n_msgs_received_);
  frequency_.std = std::sqrt(((n_msgs_received_ - 1) * std::pow(frequency_.std,2)
                              + (frequency - frequency_.avg) * (frequency - frequency_avg_last)) / n_msgs_received_);
  frequency_.min = (frequency < frequency_.min)?(frequency):(frequency_.min);
  frequency_.max = (frequency > frequency_.max)?(frequency):(frequency_.max);

  // Store this sample
  StatisticsTracker::MessageSample msg_sample = {
    header.seq,
    header.stamp,
    recv_time,
    (recv_time-latest_sample_.recv_time)};
  samples_.push_back(msg_sample);
  latest_sample_ = msg_sample;
  
  // Store the first sample, initialize the latency bounds
  if(n_msgs_received_ == 1) {
    first_sample_ = msg_sample;
    latency_.min = latency;
    latency_.max = latency;
  } else if(n_msgs_received_ == 2) {
    frequency_.min = frequency;
    frequency_.max = frequency;
  }

  // Check if we have an unlimited latency buffer
  if(sample_buffer_duration_ > ros::Duration(0.0)) {
    // Free samples outside of the window
    while(samples_.front().send_time < recv_time - sample_buffer_duration_) {
      samples_.pop_front();
    }
  }

}

double StatisticsTracker::msg_loss(const bool all_time) const
{
  double n_msgs_received = 0;
  double n_msgs_in_interval = 0;

  // Count the msgs received and the msgs we should have received
  if(all_time || sample_buffer_duration_ == ros::Duration(0.0)) {
    n_msgs_in_interval = double(latest_sample_.seq - first_sample_.seq);
    n_msgs_received = double(n_msgs_received_);
  } else {
    n_msgs_received = double(samples_.size());
    n_msgs_in_interval = double(samples_.back().seq - samples_.front().seq);
  }

  // We don't like dividing by zero
  if(n_msgs_in_interval > 0) {
    return 1.0 - (n_msgs_received / n_msgs_in_interval);
  }

  return std::numeric_limits<double>::quiet_NaN();
}

double StatisticsTracker::frequency_avg(const bool all_time) const
{
  // Return the running frequency average
  if(all_time || sample_buffer_duration_ == ros::Duration(0.0)) {
    return frequency_.avg;
  } else {
    // Count the msgs received and the length of the sampling duration
    if(samples_.size() > 0) {
      double sampling_duration = (samples_.back().recv_time - samples_.front().recv_time).toSec();

      if(sampling_duration > 0) {
        return double(samples_.size()) / sampling_duration;
      }
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

double StatisticsTracker::frequency_std(const bool all_time) const
{
  if(all_time && sample_buffer_duration_ > ros::Duration(0.0)) {
    return frequency_.std;
  } else {
    if(samples_.size() > 1) {
      double recent_frequency_squares = 0.0;
      double recent_frequency_avg = this->frequency_avg(false);

      for(std::list<MessageSample>::const_iterator sample = samples_.begin()++;
          sample != samples_.end();
          ++sample)
      {
        recent_frequency_squares += std::pow(1.0/sample->time_sep.toSec(), 2);
      }

      return std::sqrt(recent_frequency_squares/double(samples_.size()) - std::pow(recent_frequency_avg,2));
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

double StatisticsTracker::frequency_min(const bool all_time) const
{
  if(all_time || sample_buffer_duration_ == ros::Duration(0.0)) {
    return frequency_.min;
  } else {
    if(samples_.size() > 0) {
      StatisticsTracker::MessageSample min_sample =
        *std::min_element(samples_.begin(),
                          samples_.end(),
                          StatisticsTracker::frequency_cmp);
      return 1.0/(min_sample.recv_time - min_sample.send_time).toSec();
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

double StatisticsTracker::frequency_max(const bool all_time) const
{
  if(all_time || sample_buffer_duration_ == ros::Duration(0.0)) {
    return frequency_.max;
  } else {
    if(samples_.size() > 0) {
      StatisticsTracker::MessageSample max_sample =
        *std::max_element(samples_.begin(),
                          samples_.end(),
                          StatisticsTracker::frequency_cmp);
      return 1.0/(max_sample.recv_time - max_sample.send_time).toSec();
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

double StatisticsTracker::latency_avg(const bool all_time) const 
{
  if(all_time && sample_buffer_duration_ > ros::Duration(0.0)) {
    return latency_.avg;
  } else {
    if(samples_.size() > 0) {
      ros::Duration total_recent_latency = ros::Duration(0.0);

      BOOST_FOREACH(StatisticsTracker::MessageSample sample, samples_) {
        total_recent_latency += (sample.recv_time - sample.send_time);
      }

      return total_recent_latency.toSec() / double(samples_.size());
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

double StatisticsTracker::latency_std(const bool all_time) const
{
  if(all_time && sample_buffer_duration_ > ros::Duration(0.0)) {
    return latency_.std;
  } else {
    if(samples_.size() > 0) {
      double recent_latency_squares = 0.0;
      for(std::list<MessageSample>::const_iterator sample = samples_.begin();
          sample != samples_.end();
          ++sample)
      {
        recent_latency_squares += std::pow((sample->recv_time - sample->send_time).toSec(), 2);
      }
      return std::sqrt(recent_latency_squares/double(samples_.size()) - std::pow(this->latency_avg(false),2));
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

double StatisticsTracker::latency_min(const bool all_time) const
{
  if(all_time || sample_buffer_duration_ == ros::Duration(0.0)) {
    return latency_.min;
  } else {
    if(samples_.size() > 0) {
      StatisticsTracker::MessageSample min_sample =
        *std::min_element(samples_.begin(),
                          samples_.end(),
                          StatisticsTracker::latency_cmp);
      return (min_sample.recv_time - min_sample.send_time).toSec();
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}


double StatisticsTracker::latency_max(const bool all_time) const
{
  if(all_time || sample_buffer_duration_ == ros::Duration(0.0)) {
    return latency_.max;
  } else {
    if(samples_.size() > 0) {
      StatisticsTracker::MessageSample max_sample =
        *std::max_element(samples_.begin(),
                          samples_.end(),
                          StatisticsTracker::latency_cmp);
      return (max_sample.recv_time - max_sample.send_time).toSec();
    }
  }

  return std::numeric_limits<double>::quiet_NaN();
}

double StatisticsTracker::latency_latest() const 
{
  return (latest_sample_.recv_time - latest_sample_.send_time).toSec();
}

void StatisticsTracker::enable(const bool enable)
{
  enabled_ = enable;
}

bool StatisticsTracker::is_enabled() const
{
  return enabled_;
}

void StatisticsTracker::reset()
{
  latency_ = lcsr_nettools::StatisticsMeasurements();
  frequency_ = lcsr_nettools::StatisticsMeasurements();
  n_msgs_received_ = 0;
  n_msgs_out_of_order_ = 0;
  samples_.clear();
}

void StatisticsTracker::set_topic(const std::string topic_name)
{
  topic_name_ = topic_name;
  this->reset();
}

std::string StatisticsTracker::get_topic() const
{
  return topic_name_;
}

void StatisticsTracker::set_window_duration(const ros::Duration sample_buffer_duration)
{
  sample_buffer_duration_ = sample_buffer_duration;
  this->reset();
}

double StatisticsTracker::get_window_duration() const
{
  return sample_buffer_duration_.toSec();
}

void StatisticsTracker::fill_measurement_msg(lcsr_nettools::TopicMeasurements &msg, const bool all_time) const
{
  if(all_time) {
    msg.first_recv_time = first_sample_.recv_time;
  } else {
    msg.first_recv_time = samples_.back().recv_time;
  }

  msg.latest_recv_time = latest_sample_.recv_time;

  msg.msg_loss = this->msg_loss(all_time);
  msg.frequency.avg = this->frequency_avg(all_time);
  msg.frequency.std = this->frequency_std(all_time);
  msg.frequency.min = this->frequency_min(all_time);
  msg.frequency.max = this->frequency_max(all_time);
  msg.latency.avg = this->latency_avg(all_time);
  msg.latency.std = this->latency_std(all_time);
  msg.latency.min = this->latency_min(all_time);
  msg.latency.max = this->latency_max(all_time);
}


void StatisticsTracker::publish(bool throttle, double throttle_factor) const
{
  static lcsr_nettools::TopicStatistics stat_msg;

  if(!enabled_) {
    ROS_DEBUG("Request to publish, but StatisticsTracker is disabled.");
    return;
  }

  // Don't publish if we're throttling and haven't passed the throttle timestep
  if(throttle && (ros::Time::now() - stat_msg.header.stamp) < (sample_buffer_duration_ * throttle_factor)) {
    ROS_DEBUG("Request to publish, but throttling instead.");
    return;
  }

  stat_msg.header.seq ++;
  stat_msg.header.stamp = ros::Time::now();

  stat_msg.topic_name = topic_name_;

  stat_msg.recent_buffer_duration = sample_buffer_duration_;

  this->fill_measurement_msg(stat_msg.recent, false);
  this->fill_measurement_msg(stat_msg.all_time, true);

  diagnostics_pub_.publish(stat_msg);
}
