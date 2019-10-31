macro(build_example example_name example_files)
    add_executable(${example_name} ${${example_files}})
    target_include_directories(${example_name} PUBLIC ../include)
    target_link_libraries(${example_name} LINK_PUBLIC ${PROJECT_NAME} )

endmacro()

set(TEST1_FILES example1/main.cpp)
build_example(example1 TEST1_FILES)