# ROS1_Fast-DDS
A package for implementing /map topic forwarding using fast-dds on ros noetic (ubuntu 20.04 ROS1)  

fast_dds_map:  
  A ros package, placed in the "src" directory under the workspace.  
  build by catkin:  
    1. catkin_make  
    
fast_dds_map_sub:  
A test program on windows.  
  build by cmake:  
    1. mkdir build  
    2. cd build  
    3. cmake -G "Visual Studio 17 2022" ..  (Also works with visual studio 2017 and visual studio 2019)  
    4. cmake --build .  
