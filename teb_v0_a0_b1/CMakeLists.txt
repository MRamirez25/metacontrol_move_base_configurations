cmake_minimum_required(VERSION 2.8.3) 
project(teb_v0_a0_b1) 
find_package(catkin REQUIRED) 
catkin_package() 
### INSTALL ### 
install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} 
) 
