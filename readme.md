LCSR NetTools
=============

This package contains ROS libraries and interfaces for measuring network
statistics without introduction additional network overhead. 

C++ Libraries
-------------

One of the main libraries in this package is `liblcsr_nettools` which provides
an API for measuring the packet loss and min/max/avg/var latency of a topic with
messages with ROS Headers.

#### lcsr_nettools::StatisticsTracker

This class

Use from Outside of C++
-----------------------

RViz Plugins
------------

This package also includes RViz plugins which incorporate liblcsr_nettools and
report diagnostics on visualization data.

These plugins include:

* lcsr_nettools/Camera: Standard rviz/Camera plugin with statistics

These plugins are loaded automatically when RViz starts. To use them, add them
like you would any other display or panel element.
