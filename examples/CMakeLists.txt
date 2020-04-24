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

macro(build_example example_name example_files)
    add_executable(${example_name} ${${example_files}})
    target_include_directories(${example_name} PUBLIC ../include)
    target_link_libraries(${example_name} LINK_PUBLIC ${PROJECT_NAME} )
endmacro()

set(TEST1_FILES example1/main.cpp)
build_example(example1 TEST1_FILES)

set(segmentation_files segmentation/segmentation.cpp)
build_example(segmentation segmentation_files)

set(clustering_files clustering/euclidean_clusters.cpp)
build_example(clustering clustering_files)

set(filters_files filters/passthrough_filter.cpp)
build_example(filters filters_files)

set(viewer_files viewer/viewer.cpp)
build_example(viewer viewer_files)

if(${OpenCV_FOUND})
    set(grabcut_files grabcut/grabcut.cpp)
    build_example(grabcut grabcut_files)
endif()