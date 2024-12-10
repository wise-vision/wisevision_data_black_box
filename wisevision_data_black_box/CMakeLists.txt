cmake_minimum_required(VERSION 3.8)
project(wisevision_data_black_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})

include(FetchContent)
fetchcontent_declare(
    jwt-cpp 
    GIT_REPOSITORY https://github.com/Thalhammer/jwt-cpp.git
    GIT_TAG 08bcf77a687fb06e34138e9e9fa12a4ecbe12332 # v0.7.0 release
)
set(JWT_BUILD_EXAMPLES OFF CACHE BOOL "disable building examples" FORCE)
fetchcontent_makeavailable(jwt-cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lora_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wisevision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(CURL REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

include_directories(include)
include_directories(${CMAKE_BINARY_DIR})


add_library(black_box_lib
  src/zenoh_extractor.cpp
)
ament_target_dependencies(black_box_lib rclcpp std_msgs lora_msgs wisevision_msgs sensor_msgs CURL)
target_link_libraries(black_box_lib ${JSONCPP_LIBRARIES} jwt-cpp::jwt-cpp)

add_executable(black_box src/main.cpp)

target_link_libraries(black_box black_box_lib)

ament_target_dependencies(black_box rclcpp std_msgs lora_msgs wisevision_msgs sensor_msgs CURL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(CURL REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gtest(test_black_box test/black_box_test.cpp)
  target_include_directories(test_black_box PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_black_box black_box_lib)
  ament_target_dependencies(test_black_box rclcpp std_msgs lora_msgs wisevision_msgs sensor_msgs CURL)
endif()

install(TARGETS
  black_box
  DESTINATION lib/${PROJECT_NAME} 
)

ament_package()