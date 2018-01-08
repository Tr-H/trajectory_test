cmake_minimum_required(VERSION 2.8.3)
project(trajectory_test)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(mav_trajectory_generation REQUIRED)
include_directories(${mav_trajectory_generation_INCLUDE_DIRS})
link_directories(${mav_trajectory_generation_LIBARIES})
find_package(mav_trajectory_generation_ros REQUIRED)
include_directories(${mav_trajectory_generation_ros_INCLUDE_DIRS})
link_directories(${mav_trajectory_generation_ros_LIBRARIES}) 

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
      
#)
#target_link_libraries(${PROJECT_NAME} ${mav_trajectory_generation_ros_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(test1 
  src/test1.cpp
)
target_link_libraries(test1 ${mav_trajectory_generation_LIBARIES} )
target_link_libraries(test1 ${mav_trajectory_generation_ros_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
