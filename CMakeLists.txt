cmake_minimum_required(VERSION 3.10)

project(MyLib VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set the project structure
set(PROJECT_SOURCE_DIR  src)
set(PROJECT_INCLUDE_DIR include)
set(PROJECT_TEST_DIR test)

# Add library source files
set(LIB_SOURCES
    ${PROJECT_SOURCE_DIR}/odrive_can.cpp
)

# Add library header files
set(LIB_HEADERS
    ${PROJECT_INCLUDE_DIR}/odrive_can.hpp
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

# Link the test executable with your library and GTest
target_link_libraries(test_main my_lib ${GTEST_BOTH_LIBRARIES})

# Enable testing
enable_testing()

# Add the test to the CTest framework
add_test(NAME my_lib_test COMMAND test_main)