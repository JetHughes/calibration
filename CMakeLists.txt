CMAKE_MINIMUM_REQUIRED(VERSION "3.12")

# set(OpenCV_DIR "C:/OpenCV/opencv/build")

project(Calibration)

find_package(OpenCV CONFIG REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(COPY new-images2 DESTINATION images)

add_executable(calibration
    calibrate.cpp)

target_link_libraries(calibration
    ${OpenCV_LIBS}
)