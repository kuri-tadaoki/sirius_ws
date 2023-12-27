cmake_minimum_required(VERSION 3.5)
project(sirius_bringup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED system COMPONENTS)

ament_auto_find_build_dependencies()


install(DIRECTORY launch
  DESTINATION share/sirius_bringup
)

install(DIRECTORY config
  DESTINATION share/sirius_bringup
)

install(DIRECTORY rviz2
  DESTINATION share/sirius_bringup
)

install(DIRECTORY maps
  DESTINATION share/sirius_bringup
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
