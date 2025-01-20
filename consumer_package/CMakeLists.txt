cmake_minimum_required(VERSION 3.5)
project(consumer_project)

find_package(pure_prusuit REQUIRED)
find_package(rclcpp REQUIRED)
add_executable(consumer_app main.cpp)
# Debug all properties of the library target

# target_include_directories(consumer_app PUBLIC ${my_library_INCLUDE_DIRS})
# target_link_libraries(consumer_app ${my_library_LIBRARIES})
target_link_libraries(consumer_app pure_prusuit::pure_prusuit)
install(TARGETS consumer_app )