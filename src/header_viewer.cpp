
#include <iostream>

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Header.h>
#include <sstream>

void header_viewer(const topic_tools::ShapeShifter::ConstPtr msg) {

  //std::stringstream ioss;
  uint8_t* data = new uint8_t[msg->size()];
  ros::serialization::OStream stream(data, msg->size());
  msg->write(stream);

  std_msgs::Header header_sample;
  uint32_t header_size = ros::serialization::serializationLength(header_sample);
  uint8_t* header_data = new uint8_t[header_size];
  memcpy(header_data, data, sizeof(uint8_t)*header_size);
  ros::serialization::IStream header_stream(header_data, header_size);
  
  std::cerr<<"stream length: "<<header_stream.getLength()<<std::endl;

  boost::shared_ptr<std_msgs::Header> header(new std_msgs::Header);
  ros::serialization::deserialize(header_stream, *header);
  std::cerr<<*header<<std::endl;
#if 0
  boost::shared_ptr<topic_tools::ShapeShifter> ss(new topic_tools::ShapeShifter);
  ss->read(header_stream);
  ss->morph(ros::message_traits::MD5Sum<std_msgs::Header>::value(),
           "std_msgs/Header",
           ros::message_traits::Definition<std_msgs::Header>::value(),
           "false");
  std::cerr<<"ss length: "<<ss->size()<<std::endl;
  boost::shared_ptr<std_msgs::Header> header = ss->instantiate<std_msgs::Header>();
  std::cerr<<"written."<<std::endl;
#endif
#if 0
  ros::serialization::serialize(stream, *msg);

  std::cout<<"frame_id: "<<header.frame_id<<std::endl;

  //ros::OStream stream_header(data, 

  //topic_tools::ShapeShifter ss;
  //std_msgs::Header header;
  //ss.read(stream);

  //boost::shared_ptr<std_msgs::Header> header = ss.instantiate<std_msgs::Header>();

  //std::cout<<*header<<std::endl;
#endif
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "header_viewer");

  if(argc < 2) {
    return -1;
  }

  ros::NodeHandle nh;
  ros::Subscriber header_sub = nh.subscribe<topic_tools::ShapeShifter>(argv[1], 1, &header_viewer);

  ros::spin();

  return 0;
}
