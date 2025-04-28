cmake_minimum_required(VERSION 3.5)

project(test_dqrobotics)

set(CMAKE_CXX_STANDARD 11)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ADD_COMPILE_OPTIONS(-Werror=return-type
                    -Wall -Wextra -Wmissing-declarations
                    -Wredundant-decls -Woverloaded-virtual)

       
add_executable(${PROJECT_NAME}
               ${PROJECT_NAME}.cpp)

### Target the DQ Robotics library. 
### Since we updated the LD_LIBRARY_PATH, LIBRARY_PATH, and CPATH, the following target works 

# target_link_libraries(${PROJECT_NAME}
#               dqrobotics)

### However, in the case the user has the DQ Robotics library installed globally (i.e., /usr/local/lib/) and locally (i.e., ~/opt/lib.), 
### we need to ensure we use the version we want, which in this example is the one installed locally.
### Find the DQ Robotics library installed in ~/opt/lib. 
find_library(local_dqrobotics 
            NAMES dqrobotics
            PATHS ~/opt/lib
            NO_DEFAULT_PATH
            )  
if (local_dqrobotics)
    message(STATUS "Local DQ Robotics installed in ${local_dqrobotics}")
    target_link_libraries(${PROJECT_NAME}
                            ${local_dqrobotics})
else()
    message(FATAL_ERROR "Local DQ Robotics not found!")
endif()
