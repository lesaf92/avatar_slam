cmake_minimum_required(VERSION 3.16)
project(avatar_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(CTest)  # defines BUILD_TESTING (ON by default)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(avatar_core
  src/codec.cpp
  src/frames.cpp
)
add_library(avatar::avatar_core ALIAS avatar_core)
target_include_directories(avatar_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(avatar_core PUBLIC Eigen3::Eigen)
target_compile_features(avatar_core PUBLIC cxx_std_17)
target_compile_options(avatar_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
set_target_properties(avatar_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS avatar_core EXPORT avatar_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT avatar_coreTargets NAMESPACE avatar::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/avatar_core)
configure_package_config_file(cmake/avatar_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/avatar_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/avatar_core)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/avatar_coreConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/avatar_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/avatar_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/avatar_core)
install(FILES package.xml DESTINATION share/avatar_core)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  find_package(nlohmann_json 3 REQUIRED)
  add_executable(avatar_core_tests test/test_frames.cpp test/test_codec.cpp)
  target_link_libraries(avatar_core_tests PRIVATE avatar::avatar_core GTest::gtest_main
    nlohmann_json::nlohmann_json)
  target_compile_options(avatar_core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_compile_definitions(avatar_core_tests PRIVATE
    AVATAR_TESTDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../testdata")
  include(GoogleTest)
  gtest_discover_tests(avatar_core_tests)
endif()
