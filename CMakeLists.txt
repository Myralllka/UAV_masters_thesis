cmake_minimum_required(VERSION 3.1.2)
project(masters)

set(FILESNAME Masters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -Werror -Werror=vla -O3)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        mrs_lib
        nav_msgs
        geometry_msgs
        visualization_msgs
        image_geometry
        dynamic_reconfigure
        lidar_tracker
        )

generate_dynamic_reconfigure_options(
        config/Dynrec.cfg
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp mrs_lib
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
        )

add_library(${FILESNAME} src/${FILESNAME}.cpp)

add_dependencies(${FILESNAME}
        ${${FILESNAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg)

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
target_link_libraries(${FILESNAME} ${OpenCV_LIBRARIES})
target_link_libraries(${FILESNAME} ${Eigen_LIBRARIES})