find_library(my_local_dqrobotics 
        NAMES dqrobotics
        PATHS ~/opt/lib
        NO_DEFAULT_PATH
        )  
if (my_local_dqrobotics)
    message(STATUS "Local DQ Robotics installed in ${my_local_dqrobotics}")
target_link_libraries(${PROJECT_NAME}
                        ${my_local_dqrobotics})
else()
    message(FATAL_ERROR "Local DQ Robotics not found!")
endif() 
