#  Software License Agreement (BSD License)
#  Copyright (c) 2019-2023, AMBF.
#  (https://github.com/WPI-AIM/ambf)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of authors nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: Hisashi Ishida $


cmake_minimum_required (VERSION 3.3)
project (ambf_registration_plugin)

set(CMAKE_CXX_STANDARD 11)

find_package(AMBF)
find_package(Boost COMPONENTS program_options filesystem)

set (ROS1_depends ambf_server geometry_msgs sensor_msgs)

find_package (catkin QUIET
  COMPONENTS ${ROS1_depends})

if (catkin_FOUND)
    message ("Found ROS 1")
    set (AMBF_ROS_VERSION 1)
    include_directories(include ${catkin_INCLUDE_DIRS})

    catkin_package(
      INCLUDE_DIRS .
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS
    )
    message("---> LIBS " ${catkin_INCLUDE_DIRS})
    message("---> FOUND ? " ${catkin_vdrilling_msgs_FOUND})

else(catkin_FOUND)
  # look for ROS2
  set (AMBF_ROS_VERSION 2)
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
      set (AMBF_ROS_VERSION 2)
      message ("Found ROS 2")
      set (ROS2_depends rclcpp geometry_msgs sensor_msgs ambf_server AMBF)
      foreach (_dep ${ROS2_depends})
        find_package (${_dep} REQUIRED)
      endforeach()
  endif (ament_cmake_FOUND)
endif(catkin_FOUND)

include_directories(${Boost_INCLUDE_DIRS})

link_directories(${AMBF_LIBRARY_DIRS})
add_definitions(${AMBF_DEFINITIONS})
include_directories(${AMBF_INCLUDE_DIRS})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS} )
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# For registration plugin
add_library(registration_plugin SHARED
    src/plugin/registration_plugin.cpp
    src/plugin/registration_plugin.h
    src/utils/camera_panel_manager.cpp
    src/utils/camera_panel_manager.h
    src/utils/CRTKInterface.cpp
    src/utils/CRTKInterface.h
    src/registration/point_cloud_registration.cpp
    src/registration/point_cloud_registration.h
    src/registration/pivot_calibration.cpp
    src/registration/pivot_calibration.h
    src/registration/hand_eye_calibration.cpp
    src/registration/hand_eye_calibration.h
    )


if (AMBF_ROS_VERSION EQUAL 1)
    add_dependencies (registration_plugin ${catkin_EXPORTED_TARGETS})
    target_link_libraries (registration_plugin ${Boost_LIBRARIES} ${AMBF_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
else ()
 target_link_libraries(registration_plugin
        ${Boost_LIBRARIES}
        ${AMBF_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )
  install (
    TARGETS registration_plugin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )
  ament_target_dependencies (registration_plugin ${ROS2_depends})
  ament_package ()
endif()

# set_property(TARGET registration_plugin PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# add_executable(TestHandEye src/registration/hand_eye_calibration.cpp)
# target_link_libraries(TestHandEye ${AMBF_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(TestPointCloud src/registration/point_cloud_registration.cpp)
target_link_libraries(TestPointCloud ${AMBF_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})