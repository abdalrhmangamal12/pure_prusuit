
cmake_minimum_required(VERSION 3.22)
project("pure_prusuit")
add_executable(main src/main.cpp ) # target
add_library(pure_prusuit_lib SHARED src/ppcontroller.cpp)
target_include_directories(pure_prusuit_lib PUBLIC include/ )

target_link_libraries(main pure_prusuit_lib)
install(TARGETS main pure_prusuit_lib DESTINATION ) 


include(CTest)
find_package(GTest REQUIRED)
add_executable(main_test test/main_test.cpp src/ppcontroller.cpp )
target_include_directories(main_test PUBLIC include/)
target_link_libraries(main_test  gtest  gtest_main )
add_test(NAME main_testing COMMAND main_test)

