cmake_minimum_required(VERSION 3.10)

project(MyLib VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set the project structure
set(PROJECT_SOURCE_DIR  src)
set(PROJECT_INCLUDE_DIR include)
set(PROJECT_TEST_DIR test)
set(PROJECT_SCRIPTS_DIR scripts)  # New scripts folder 

include(CheckIncludeFile)
check_include_file("linux/can.h" HAVE_LINUX_CAN_H)

if (HAVE_LINUX_CAN_H)
    message(STATUS "Found linux can headers, enabling.")
else()
    message(STATUS "Linux can headers not found, disabling")
endif()

# Add library source files
set(LIB_SOURCES
    ${PROJECT_SOURCE_DIR}/odrive_can_cpp.cpp
)

# Add library header files
set(LIB_HEADERS
    ${PROJECT_INCLUDE_DIR}/odrive_can_cpp.hpp
)

# Create the library
add_library(my_lib ${LIB_SOURCES} ${LIB_HEADERS})

# Set include directories
target_include_directories(my_lib PUBLIC ${PROJECT_INCLUDE_DIR})

# Find and link against GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Add test executable
add_executable(test_main ${PROJECT_TEST_DIR}/main.cpp)


# Link the test executable with your library, GTest, and pthread
target_link_libraries(test_main my_lib ${GTEST_BOTH_LIBRARIES} pthread)


# Link the test executable with your library and GTest
target_link_libraries(test_main my_lib ${GTEST_BOTH_LIBRARIES})

# Enable testing
enable_testing()

# Add the test to the CTest framework
add_test(NAME my_lib_test COMMAND test_main)

# Add scripts executable

add_executable(script_exe ${PROJECT_SCRIPTS_DIR}/script.cpp)  # Replace with your actual script file

# Link the scripts executable with your library
target_link_libraries(script_exe my_lib) 