cmake_minimum_required(VERSION 2.8.3)
project(triskar)

find_package(catkin REQUIRED COMPONENTS roscpp
					sensor_msgs
					triskar_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})


###########
## Build ##
###########

add_executable(teleop_pixy_joy src/PixyTeleop.cpp)

target_link_libraries(teleop_pixy_joy ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS teleop_pixy_joy
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
