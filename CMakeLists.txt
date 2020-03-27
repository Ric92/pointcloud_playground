##---------------------------------------------------------------------------------------------------------------------
##  POINTCLOUD_PLAYGROUND
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2019 ViGUS University of Seville
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------


cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
project(pointcloud_playground VERSION 0.1 LANGUAGES C CXX)

file(GLOB_RECURSE POINTCLOUDPLAYGROUND_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE POINTCLOUDPLAYGROUND_HEADER_FILES "include/*.h" "include/*.inl")

add_library(${PROJECT_NAME} SHARED ${POINTCLOUDPLAYGROUND_HEADER_FILES} ${POINTCLOUDPLAYGROUND_SOURCE_FILES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

## Load PCL
find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
	MESSAGE( STATUS "Adding PCL")
	target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})
	
	target_compile_definitions(${PROJECT_NAME} PUBLIC "ENABLE_PCL" "DISABLE_PCAP" "DISABLE_PNG" "DISABLE_LIBUSB_1_0")

	if(${PCL_VERSION} VERSION_GREATER 1.8)
		target_compile_definitions(${PROJECT_NAME} PUBLIC "HAS_PCL_1_8")
	else()
		target_compile_definitions(${PROJECT_NAME} PUBLIC "HAS_PCL_1_7")
	endif()

else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

find_package(OpenCV)
if(${OpenCV_FOUND})
	target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})
	target_compile_definitions(${PROJECT_NAME} PUBLIC "HAS_OPENCV")
endif()

#########################################
######			Examples 		   ######
#########################################

add_subdirectory(examples)
