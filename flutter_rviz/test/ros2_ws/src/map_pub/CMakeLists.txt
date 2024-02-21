cmake_minimum_required(VERSION 3.8)
project(map_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE) # Uncomment if within a git repo and copyright/license issues resolved
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install map files
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
