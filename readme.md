LCSR NetTools
=============

This package contains ROS libraries and interfaces for measuring network
statistics without introduction additional network overhead. 

Demonstration
-------------

To see the library in action, using TF, run:

```sh
roslaunch lcsr_nettools tf_statistics.launch
```

The following topics will be published in addition to /tf:
- `/statistics/tf_frames/robot/header_samples`
    - This contains samples of the header message from the `/robot` tf frame.
    - It is published each time the tf_statistics node receives a tf message.
- `/topic_statistics`
    - This contains the computed statistics over a window of 10 seconds, and since the start of the node.  
    - It is published every 5 seconds.


C++ Libraries
-------------

One of the main libraries in this package is `liblcsr_nettools` which provides
an API for measuring the packet loss and min/max/avg/var latency of a topic with
messages with ROS Headers.

#### lcsr_nettools::StatisticsTracker

This class is used to compute statistics for a given topic. It is intended to be used from within code which is already subscribed to a given topic, so that additional socket connections are not needed to gather information about a given data stream.

It can compute the folllowing min/max/average/var over a window and for all time:
 - Message latency
 - Message frequency

Usage of the class involves primarily two functions:
 - `StatisticsTracker::sample(...)` Sample a header data point
 - `StatisticsTracker::publish(...)` Publish the topic statistics on the `topic_statistics` topic.

You can view the statistics with `rqt_plot` or `rxplot`. In order to record the statistics, you can use rosbag to record the messages published on the `topic_statistics` topic.

See the [StatisticsTracker API](include/lcsr_nettools/statistics.h) for more information.

Use from Outside of C++
-----------------------

If you don't mind making another topic connection, you can run the `statistics_node` which subscribes to the `header_samples` topic and instantiates a `StatisticsTracker`. You can publish to `header_samples` from any other kind of ROS node, and these samples will be used to compute the statistics.

See [examples/tf_statistics.launch](examples/tf_statistics.launch) for a usage example with a python node wich publishes to `header_samples`. 

RViz Plugins
------------

This package also includes RViz plugins which incorporate liblcsr_nettools and
report diagnostics on visualization data.

These plugins include:

* lcsr_nettools/Camera: Standard rviz/Camera plugin with statistics

These plugins are loaded automatically when RViz starts. To use them, add them
like you would any other display or panel element.
