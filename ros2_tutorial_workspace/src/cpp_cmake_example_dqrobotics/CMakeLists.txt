cmake_minimum_required(VERSION 3.5)

project(test_dqrobotics)

set(CMAKE_CXX_STANDARD 11)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ADD_COMPILE_OPTIONS(-Werror=return-type
                    -Wall -Wextra -Wmissing-declarations
                    -Wredundant-decls -Woverloaded-virtual)

# Set here your local library directory
set(MY_LOCAL_LIBRARIES_DIR /home/USERNAME/opt/)

INCLUDE_DIRECTORIES(${MY_LOCAL_LIBRARIES_DIR}/include)    
LINK_DIRECTORIES(${MY_LOCAL_LIBRARIES_DIR}/lib)           

add_executable(${PROJECT_NAME}
               ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
                      dqrobotics)