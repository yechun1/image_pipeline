cmake_minimum_required(VERSION 3.5)
project(stereo_image_proc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
# find_package(image_proc REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# See note in image_proc/CMakeLists.txt
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

# Nodelet library
add_library(${PROJECT_NAME} SHARED src/libstereo_image_proc/processor.cpp src/nodelets/disparity.cpp src/nodelets/point_cloud2.cpp)
target_link_libraries(${PROJECT_NAME} ${ament_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  "cv_bridge"
  "image_geometry"
#  "image_proc"
  "image_transport"
  "class_loader"
  "message_filters"
  "rclcpp"
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
)

## Standalone node
#add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
#target_link_libraries(stereoimageproc_exe stereo_image_proc)
#set_target_properties(stereoimageproc_exe PROPERTIES OUTPUT_NAME stereo_image_proc)
#install(TARGETS stereoimageproc_exe
#        DESTINATION lib/${PROJECT_NAME}
#)
#install(TARGETS stereoimageproc_exe
#        DESTINATION bin

# install the launch file
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

# install the include directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)
