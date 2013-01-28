
#include <lcsr_nettools/statistics.h>
#include <algorithm>

using namespace lcsr_nettools;

Statistics::Statistics(const ros::Duration latency_buffer_duration) :
  latency_buffer_duration_(latency_buffer_duration),
  latencies_(),
  first_sample_(),
  n_msgs_received_(0),
  n_msgs_out_of_order_(0),
  total_latency_(0.0),
  min_latency_(0.0),
  max_latency_(0.0)
{

}

Statistics::~Statistics() {

}

void Statistics::sample(const std_msgs::Header &header,
                   const ros::Time &sample_time)
{
  // Check if this message is out of order
  if(header.seq < latencies_.back().seq) {
    n_msgs_out_of_order_++;
  }

  // Compute the latency
  ros::Duration latency = sample_time - header.stamp;

  // Compute running total latency, min latency, max latency
  total_latency_ += latency;
  min_latency_ = (latency < min_latency_)?(latency):(min_latency_);
  max_latency_ = (latency > max_latency_)?(latency):(max_latency_);

  // Store this sample
  Statistics::Statistics::MessageSample msg_sample = {header.seq, header.stamp, sample_time};
  latencies_.push_back(msg_sample);
  latest_sample_ = msg_sample;
  
  // Store the first sample, initialize the latency bounds
  if(n_msgs_received_ == 0) {
    first_sample_ = msg_sample;
    min_latency_ = latency;
    max_latency_ = latency;
  }

  // Increment the sample counter
  n_msgs_received_++;
  
  // Check if we have an unlimited latency buffer
  if(latency_buffer_duration_ > ros::Duration(0.0)) {
    // Free samples outside of the window
    while(latencies_.front().send_time < sample_time - latency_buffer_duration_) {
      latencies_.pop_front();
    }
  }

}

double Statistics::packet_loss(const bool recent) const
{
  if(recent && latency_buffer_duration_ > ros::Duration(0.0)) {
    if(latencies_.size() > 0) {
      return 1.0 -
        double(latencies_.back().seq - latencies_.front().seq)
        / double(latencies_.size());
    }
  } else {
    if(n_msgs_received_ > 0) {
      return 1.0 -
        double(latest_sample_.seq - first_sample_.seq)
        / double(n_msgs_received_);
    }
  }

  return 0.0;
}

ros::Duration Statistics::min_latency(const bool recent) const
{
  if(recent && latency_buffer_duration_ > ros::Duration(0.0)) {
    if(latencies_.size() > 0) {
      Statistics::MessageSample min_sample = *std::min_element(latencies_.begin(), latencies_.end(), Statistics::latency_cmp);
      return min_sample.recv_time - min_sample.send_time;
    }
  } else {
    return min_latency_;
  }

  return ros::Duration(0.0);
}

ros::Duration Statistics::max_latency(const bool recent) const
{
  if(recent && latency_buffer_duration_ > ros::Duration(0.0)) {
    if(latencies_.size() > 0) {
      Statistics::MessageSample max_sample = *std::max_element(latencies_.begin(), latencies_.end(), Statistics::latency_cmp);
      return max_sample.recv_time - max_sample.send_time;
    }
  } else {
    return max_latency_;
  }

  return ros::Duration(0.0);
}

ros::Duration Statistics::average_latency(const bool recent) const 
{
  if(recent && latency_buffer_duration_ > ros::Duration(0.0)) {
    ros::Duration total_recent_latency = ros::Duration(0);
    for(std::list<Statistics::MessageSample>::const_iterator it = latencies_.begin();
        it != latencies_.end();
        ++it)
    {
      total_recent_latency += (it->recv_time - it->send_time);
    }
    return total_recent_latency * (1.0 / double(latencies_.size()));
  } else {
    if(n_msgs_received_ > 0) {
      return total_latency_ * (1.0 / double(n_msgs_received_));
    }
  }

  return ros::Duration(0.0);
}

ros::Duration Statistics::latest_latency() const 
{
  return latest_sample_.recv_time - latest_sample_.send_time;
}
