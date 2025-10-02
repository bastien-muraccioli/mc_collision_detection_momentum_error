cmake_minimum_required(VERSION 3.10)

project(
  CollisionDetectionMomentumError
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
configure_file(etc/CollisionDetectionMomentumError.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/CollisionDetectionMomentumError.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/CollisionDetectionMomentumError.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC 
src/CollisionDetectionMomentumError.cpp
src/LpfThreshold.cpp
)

set(plugin_HDR 
src/CollisionDetectionMomentumError.h
src/LpfThreshold.h
)

# The add_plugin macro is provided by mc_rtc
add_plugin(CollisionDetectionMomentumError "${plugin_SRC}" "${plugin_HDR}")

# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()
