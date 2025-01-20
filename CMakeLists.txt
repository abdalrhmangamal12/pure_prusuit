
cmake_minimum_required(VERSION 3.22)
project("pure_prusuit")
add_executable(main src/main.cpp ) # target
add_library(pure_prusuit_lib SHARED src/ppcontroller.cpp)
target_include_directories(pure_prusuit_lib PUBLIC include/ )

target_link_libraries(main pure_prusuit_lib)
install(TARGETS main pure_prusuit_lib DESTINATION ) 