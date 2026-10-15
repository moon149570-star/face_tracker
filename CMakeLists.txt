cmake_minimum_required(VERSION 3.10)

project(face_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS
    core
    videoio
    imgproc
    imgcodecs
    objdetect
)

add_executable(camera_probe
    src/main.cpp
)

target_include_directories(camera_probe PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_probe PRIVATE
    ${OpenCV_LIBS}
    pthread
)
