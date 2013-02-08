
#include <boost/python.hpp>

#include <lcsr_nettools/statistics.h>
#include <ros/ros.h>

using namespace boost::python;
using namespace lcsr_nettools;

struct rosTime_from_python_genpyTime {

  rosTime_from_python_genpyTime()
  {
    boost::python::converter::registry::push_back(&convertible,
                                                  &construct,
                                                  boost::python::type_id<ros::Time>());
  }

  static void* convertible(PyObject* obj_ptr) {
    if(PyObject_HasAttrString(obj_ptr, "secs")
       && PyInt_Check(PyObject_GetAttrString(obj_ptr, "secs"))
       && PyObject_HasAttrString(obj_ptr, "nsecs")
       && PyInt_Check(PyObject_GetAttrString(obj_ptr, "nsecs")))
    {
      ROS_INFO("Convertible...");
      return obj_ptr;
    }

    ROS_INFO("NOT Convertible...");
    return 0;
  }

  static void construct(PyObject* obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    ROS_INFO("Trying to construct...");
    uint32_t secs = PyInt_AsUnsignedLongMask(PyObject_GetAttrString(obj_ptr, "secs"));
    uint32_t nsecs = PyInt_AsUnsignedLongMask(PyObject_GetAttrString(obj_ptr, "nsecs"));

    void* storage =
      ((boost::python::converter::rvalue_from_python_storage<ros::Time>*)
       data)->storage.bytes;

    new (storage) ros::Time(secs, nsecs);

    data->convertible = storage;
  }

};

class StatisticsTrackerWrapper : public StatisticsTracker {
public:
  StatisticsTrackerWrapper(std::string topic_name, double buffer_duration):
    StatisticsTracker(ros::NodeHandle(), topic_name, ros::Duration(buffer_duration))
  { 
  }

  void sample(object py_header, object py_sample_time) {
    std_msgs::Header header;
    header.seq = extract<uint32_t>(py_header.attr("seq"));
    header.stamp = extract<ros::Time>(py_header.attr("stamp"));
    header.frame_id = extract<std::string>(py_header.attr("frame_id"));

    ros::Time sample_time = extract<ros::Time>(py_header.attr("stamp"));

    ROS_INFO("Calling sample...");

    StatisticsTracker::sample(header);
  }

  void publish() {
    StatisticsTracker::publish();
    ros::spinOnce();
  }
};

BOOST_PYTHON_MODULE(libstatistics)
{

  int argc = 0;
  char **argv = 0;
  ros::init(argc, argv, "statistics_tracker");

  class_<StatisticsTrackerWrapper>("StatisticsTracker", init<std::string, double>())
    .def("sample", &StatisticsTrackerWrapper::sample)
    .def("enable", &StatisticsTrackerWrapper::enable)
    .def("reset", &StatisticsTrackerWrapper::reset)
    .def("publish", &StatisticsTrackerWrapper::publish)
    ;

  rosTime_from_python_genpyTime();
}

