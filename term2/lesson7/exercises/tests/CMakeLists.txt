# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/exercise1)
include_directories(${PROJECT_SOURCE_DIR}/src/Eigen)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()



#set(test_SOURCES RunAllTests.cpp 
#	testUKF.cpp ../src/exercise1/ukf.cpp) 

#add_executable(RunAllTests ${test_SOURCES})
#target_link_libraries(RunAllTests gtest_main)
#add_test(NAME RunAllTests COMMAND RunAllTests)
