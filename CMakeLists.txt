project(4D_Segmentation)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

IF(WIN32 OR win64)
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .lib .dll)
ELSE()
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .a)
ENDIF()


find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )
 

add_subdirectory(src)

include_directories(C:/opencv/build)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${4D_Segmentation_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})

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

add_library (4D_Segmentation src/4D_Segmentation.cpp src/RegionTree.cpp src/GraphSegmentation.cpp src/OpticalFlow.cpp ${4D_Segmentation_HEADERS})
target_link_libraries (4D_Segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (Test_Seg src/TestVideoSegmentation.cpp ${4D_Segmentation_HEADERS})
target_link_libraries (Test_Seg 4D_Segmentation ${PCL_LIBRARIES} ${OpenCV_LIBS})
