cmake_minimum_required(VERSION 3.8)
project(simplesim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Wpessimizing-move)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fmt REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(SFML REQUIRED COMPONENTS graphics system window)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(simplesim_node src/main.cpp src/sim.cpp src/drone.cpp src/controller.cpp src/text.cpp src/visuals.cpp src/managed_sprite.cpp)
add_executable(simplesim_node::exe ALIAS simplesim_node)

target_include_directories(simplesim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(simplesim_node PUBLIC
    rclcpp
    geometry_msgs
    std_srvs
    nav_msgs
    tf2
    tf2_ros
    )

target_link_libraries(simplesim_node PUBLIC sfml-graphics sfml-system sfml-window fmt)
  
target_compile_features(simplesim_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Pass the project name as a compile definition
target_compile_definitions(simplesim_node PRIVATE PROJECT_NAME="${PROJECT_NAME}")

# Copy data files
file(COPY "${CMAKE_SOURCE_DIR}/data" DESTINATION "${CMAKE_BINARY_DIR}")

install(TARGETS simplesim_node
DESTINATION lib/${PROJECT_NAME})

# Install sprites
install(DIRECTORY "${CMAKE_SOURCE_DIR}/data/sprites/"
        DESTINATION "lib/${PROJECT_NAME}/data/sprites")

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()