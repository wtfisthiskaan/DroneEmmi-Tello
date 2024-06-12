cmake_minimum_required(VERSION 3.0.2)
project(example LANGUAGES CXX)

# Add the parent directory containing the tello library
add_subdirectory(.. bin)

# Set the sources
set(SOURCES
    src/main.cpp
    src/DroneEmmi.cpp
)

# Find the OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add the executable target
add_executable(example ${SOURCES})

# Include the headers
target_include_directories(example PUBLIC include)

# Link against the tello library and OpenCV libraries
target_link_libraries(example tello ${OpenCV_LIBS})

# Link against pthread if not using MSVC
if (NOT MSVC)
    target_link_libraries(example pthread)
endif()

# Set compile options for C++14 standard
if (MSVC)
    target_compile_options(example PUBLIC /std:c++14)
else()
    target_compile_options(example PUBLIC --std=c++14)
endif()

# Messages to display OpenCV configuration
message("cv_libs: ${OpenCV_LIBRARIES}")
message("cv_includes: ${OpenCV_INCLUDE_DIRS}")
