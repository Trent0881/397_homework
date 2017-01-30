cmake_minimum_required(VERSION 2.8.3)
project(taz15_p3)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

# Define .action stuff
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES commandAction.action)
generate_messages(DEPENDENCIES actionlib_msgs)


# This includes catkin_package(), right?
catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


add_executable(min_sim2 src/min_sim2.cpp)
add_executable(min_ctl2 src/min_ctl2.cpp)
add_executable(sin_command2 src/sin_command2.cpp)
add_executable(sin_command_client2 src/sin_command_client2.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(minimal_nodes_node minimal_nodes_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(min_sim2  ${catkin_LIBRARIES} )
target_link_libraries(min_ctl2  ${catkin_LIBRARIES} )
target_link_libraries(sin_command2  ${catkin_LIBRARIES} )
target_link_libraries(sin_command_client2  ${catkin_LIBRARIES} )

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)

cs_install()
cs_export()
    
