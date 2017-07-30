set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/Eigen)

set(sources main.cpp ukf.cpp)
add_executable(Exercise4 ${sources})
