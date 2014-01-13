cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )

project(4D_Segmentation)

add_subdirectory(src)

include_directories(C:/opencv/build)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(C:/blepo/external64/Microsoft/Kinect)
include_directories(${4D_Segmentation_SOURCE_DIR}/include)
include_directories(../PCL_WindowsKinectSDK/include)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(C:/blepo/external64/Microsoft/Kinect)
link_directories(../PCL_WindowsKinectSDK/build/)

add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE 4D_Segmentation_HEADERS include/*.h)
file(GLOB_RECURSE 4D_Segmentation_SOURCES src/*.cpp)
set (4D_Segmentation_INCLUDE_DIRS "")
foreach (_headerFile ${4D_Segmentation_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND 4D_Segmentation_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES 4D_Segmentation_INCLUDE_DIRS)

include_directories(${4D_Segmentation_INCLUDE_DIRS})

add_executable (4D_Segmentation ${PCL_WindowsKinectSDK_SOURCES})
target_link_libraries (4D_Segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS} Kinect10.lib debug Debug/PCL_WindowsKinectSDK.lib Release Release/PCL_WindowsKinectSDK.lib)