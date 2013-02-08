
#include <ros/ros.h>

#include <lcsr_nettools/statistics.cpp>

template<typename T>
void msg_handler(boost::shared_ptr<T> msg) {


int main(int argc, char** argv) {

  ros::init(argc, argv, "statistics_tracker");

  if(argc == 1) {
    std::cerr<<"Usage: statistics_tracker [[topic_name] ... ]"<<std::endl;
    return -1;
  }

  ros::NodeHandle nh;
  std::vector<ros::Subscriber> subscribers;

  for(int i=1; i<argc; i++) {

    subscribers.push_back(nh.subscribe(argv[i]),);
    

  

  return 0;
}
