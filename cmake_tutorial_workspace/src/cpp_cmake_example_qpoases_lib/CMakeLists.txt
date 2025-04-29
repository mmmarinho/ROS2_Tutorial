cmake_minimum_required(VERSION 3.5)

project(test_proxsuite)

set(CMAKE_CXX_STANDARD 20)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ADD_COMPILE_OPTIONS(-Werror=return-type
                    -Wall -Wextra -Wmissing-declarations
                    -Wredundant-decls -Woverloaded-virtual)

add_executable(${PROJECT_NAME}
               ${PROJECT_NAME}.cpp)
               
target_link_libraries(${PROJECT_NAME}
             qpOASES
)