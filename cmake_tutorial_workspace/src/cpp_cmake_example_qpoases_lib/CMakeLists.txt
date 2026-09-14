cmake_minimum_required(VERSION 3.5)

project(test_qpoases)

set(CMAKE_CXX_STANDARD 20)

ADD_COMPILE_OPTIONS(-Werror=return-type
                    -Wall -Wextra -Wmissing-declarations
                    -Wredundant-decls -Woverloaded-virtual)

add_executable(${PROJECT_NAME}
               src/${PROJECT_NAME}.cpp)
               
target_link_libraries(${PROJECT_NAME}
             qpOASES
)