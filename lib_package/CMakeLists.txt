
cmake_minimum_required(VERSION 3.22)
project("pure_prusuit")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# add_executable(main src/main.cpp ) # target
add_library(${PROJECT_NAME} SHARED src/ppcontroller.cpp)

# to overcome the relative path of include directories after installation
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# it is equivalent to #target_link_libraries( ${PROJECT_NAME} rclcpp )
ament_target_dependencies(${PROJECT_NAME} rclcpp )


# move the built target to installation and relate the export to this target
install(TARGETS  ${PROJECT_NAME}    EXPORT export_${PROJECT_NAME}) 

# copy include to install
install(
  DIRECTORY include/
  DESTINATION include
)
# generate pure_prusuitConfig.cmake to let other package find it using find_package(pure_prusuit required) and link against it
ament_export_targets(export_${PROJECT_NAME})

# include(CTest)
# find_package(GTest REQUIRED)
# add_executable(main_test test/main_test.cpp src/ppcontroller.cpp )
# target_include_directories(main_test PUBLIC include/)
# target_link_libraries(main_test  gtest  gtest_main )
# add_test(NAME main_testing COMMAND main_test)

# to do some extra work relate to organisations
ament_package()
