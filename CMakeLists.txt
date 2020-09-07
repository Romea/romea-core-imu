cmake_minimum_required(VERSION 2.8.3)
project(romea_imu)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS roscpp romea_common romea_common_utils)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

###########
## Build ##
###########

include_directories(
    include
    SYSTEM ${catkin_INCLUDE_DIRS}
    SYSTEM /usr/include/eigen3
    )

add_library(romea_imu
    src/IMU6DOF.cpp
    src/IMU9DOF.cpp
    src/IMUAHRS.cpp
    src/IMUVRU.cpp
    src/algorithms/ZeroVelocityEstimator.cpp
    src/algorithms/RollPitchKalmanEstimator.cpp
    src/algorithms/TriadAlgorithm.cpp
    )


## Specify libraries to link a library or executable target against
target_link_libraries(tag ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trackbod_driver trackbod_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
#if(CATKIN_ENABLE_TESTING)

#catkin_add_gtest(${PROJECT_NAME}-test test/test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
# endif()

#endif(CATKIN_ENABLE_TESTING)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
