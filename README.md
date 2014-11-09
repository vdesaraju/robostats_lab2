robostats_lab2
==============

Pointy cloud classification


To build (from the robostats_lab2 directory):
     catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

To setup workspace (from the robostats_lab2 directory):
     catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

To run:
    roslaunch classifiers test.launch 

This will run classifiers.cpp, which will load the datafile specified in the launch file and visualize it via PCL.
* You can rotate/pan/etc with the mouse+shift/ctrl/etc.
* Hit r to reset the view back to center. The initial view is garbage, so hit this after the window opens to see the cloud.
