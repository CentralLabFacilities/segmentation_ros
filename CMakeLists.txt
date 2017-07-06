cmake_minimum_required(VERSION 2.8)

project(segmentation)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    cv_bridge
    std_msgs
    pcl_ros
    pcl_conversions
    tf_conversions
    pcl_msgs
    pluginlib
    roslib
    actionlib_msgs
    actionlib
    object_recognition_msgs
    grasping_msgs
    planning_scene_manager_msgs
    message_generation)

add_action_files(DIRECTORY action FILES imageRoi.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs sensor_msgs)


#============================ c++11 =================================
#======= NOT WORKING WITH PCL/BOOST COMPILED BY C++03 COMPILER ======

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")



#--------------------------packages-----------------------------------

#Log4cxx
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Modules/FindXcvlLibrary.cmake)

FIND_PACKAGE(Log4cxx REQUIRED)
LINK_DIRECTORIES(${Log4cxx_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${Log4cxx_INCLUDE_DIRS})

# boost
FIND_PACKAGE(Boost 1.54 COMPONENTS filesystem thread program_options REQUIRED)

# opencv
FIND_PACKAGE(OpenCV 3.2.0 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

FIND_PACKAGE(PCL 1.8 QUIET REQUIRED COMPONENTS common)
if(PCL_FOUND)
  LINK_DIRECTORIES(${PCL_COMMON_LIBRARY_DIRS})
  INCLUDE_DIRECTORIES(${PCL_COMMON_INCLUDE_DIRS})
  MESSAGE(STATUS "PCL found. Version: ${PCL_VERSION}")
endif(PCL_FOUND)

FIND_PACKAGE(PCA 0.7 REQUIRED)
LINK_DIRECTORIES(${PCA_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${PCA_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
LINK_LIBRARIES(${EIGEN3_LIBRARY_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
    CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    cv_bridge
    std_msgs
    actionlib_msgs
    DEPENDS
    Boost
    PCL
    Eigen3)

INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})

SET(SEG_HEAD src/ImageRegion.h
            src/ImageSource.h
            src/Results.h
            src/Roi.h
            src/segmentation.h
            src/Surface.h)

SET(SEG_SRC src/ImageRegion.cpp
            src/ImageSource.cpp
            src/Results.cpp
            src/Roi.cpp
            src/segmentation.cpp
            src/Surface.cpp)

ADD_EXECUTABLE(Segmentation src/communicator.cpp ${SEG_SRC})
target_link_libraries(Segmentation ${PCL_LIBRARIES} ${PCA_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${OpenCV_LIBRARIES} ${ROS_LIBRARIES})
INSTALL(TARGETS Segmentation
    DESTINATION bin)

