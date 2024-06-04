option(BUILD_SHARED_LIBS "Build libraries as SHARED" ON)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Export compile_commands.json" ON)

if(${CMAKE_EXPORT_COMPILE_COMMANDS})
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()


set(CONFIG_BASENAME       "${PROJECT_NAME}Config")
set(CONFIG_FILENAME       "${CONFIG_BASENAME}.cmake")
set(VERSION_FILENAME      "${CONFIG_BASENAME}Version.cmake")
set(EXPORTED_TARGETS_NAME "${PROJECT_NAME}Targets")
set(EXPORTED_TARGETS_FILE "${EXPORTED_TARGETS_NAME}.cmake")

include(GNUInstallDirs)
set(INSTALL_BIN_DIR       "${CMAKE_INSTALL_BINDIR}")
set(INSTALL_LIB_DIR       "${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}")
set(INSTALL_INCLUDE_DIR   "${CMAKE_INSTALL_INCLUDEDIR}")
set(INSTALL_CMAKE_DIR     "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}")
set(INSTALL_PKGCONFIG_DIR "${CMAKE_INSTALL_DATADIR}/pkgconfig")

set(${PROJECT_NAME}_LIBRARIES)
set(${PROJECT_NAME}_EXECUTABLES)


macro(find_package_and_export)
  find_package(${ARGN})
  string(REPLACE ";" " " argn_with_spaces "${ARGN}")
  string(APPEND PROJECT_FIND_DEPENDENCY_CALLS
         "find_dependency("
         "${argn_with_spaces}"
         ")\n")
endmacro()

macro(link_and_include _target)
  cmake_parse_arguments(link_and_include
    "" "" "LIBRARIES;INCLUDE_DIRS" ${ARGN})
  if (DEFINED link_and_include_LIBRARIES)
    target_link_libraries(${_target} ${link_and_include_LIBRARIES})
  endif()
  if (DEFINED link_and_include_INCLUDE_DIRS)
    target_include_directories(${_target} PUBLIC ${link_and_include_INCLUDE_DIRS})
  endif()
endmacro()

macro(library _target)
  cmake_parse_arguments(library
    "" "" "LIBRARIES;INCLUDE_DIRS" ${ARGN})
  add_library(${_target} ${library_UNPARSED_ARGUMENTS})
  add_library(${PROJECT_NAME}::${_target} ALIAS ${_target})
  list(APPEND ${PROJECT_NAME}_LIBRARIES ${_target})
  link_and_include(${_target}
    LIBRARIES    ${library_LIBRARIES}
    INCLUDE_DIRS ${library_INCLUDE_DIRS})
endmacro()

macro(executable _target)
  cmake_parse_arguments(executable
    "" "" "LIBRARIES;INCLUDE_DIRS" ${ARGN})
  add_executable(${_target} ${executable_UNPARSED_ARGUMENTS})
  add_dependencies(${_target} ${PROJECT_NAME})
  list(APPEND ${PROJECT_NAME}_EXECUTABLES ${_target})
  link_and_include(${_target}
    LIBRARIES    ${executable_LIBRARIES}
    INCLUDE_DIRS ${executable_INCLUDE_DIRS})
endmacro()

macro(package)
  cmake_parse_arguments(package
    "" "INCLUDE" "LIBRARIES;INCLUDE_DIRS" ${ARGN})

  if (NOT DEFINED ${package_INCLUDE})
    set(package_INCLUDE "include")
  endif()
  foreach(_target ${${PROJECT_NAME}_LIBRARIES} ${${PROJECT_NAME}_EXECUTABLES})
    target_include_directories(${_target} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${package_INCLUDE}>
      $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>)
    link_and_include(${_target}
      LIBRARIES    ${package_LIBRARIES}
      INCLUDE_DIRS ${package_INCLUDE_DIRS})
  endforeach()

  install(TARGETS     ${${PROJECT_NAME}_LIBRARIES}
          EXPORT      ${EXPORTED_TARGETS_NAME}
          DESTINATION ${INSTALL_LIB_DIR})
  install(TARGETS     ${${PROJECT_NAME}_EXECUTABLES}
          EXPORT      ${EXPORTED_TARGETS_NAME}
          DESTINATION ${INSTALL_BIN_DIR})
  install(DIRECTORY   "${CMAKE_CURRENT_SOURCE_DIR}/${package_INCLUDE}/"
          DESTINATION ${INSTALL_INCLUDE_DIR})

  include(CMakePackageConfigHelpers)

  write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${VERSION_FILENAME}"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion)

  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${CONFIG_FILENAME}.in"
    "${PROJECT_BINARY_DIR}/${CONFIG_FILENAME}"
    INSTALL_DESTINATION ${INSTALL_CMAKE_DIR})

  install(EXPORT      ${EXPORTED_TARGETS_NAME}
          FILE        ${EXPORTED_TARGETS_FILE}
          NAMESPACE   ${PROJECT_NAME}::
          DESTINATION ${INSTALL_CMAKE_DIR})

  install(FILES
          "${PROJECT_BINARY_DIR}/${CONFIG_FILENAME}"
          "${PROJECT_BINARY_DIR}/${VERSION_FILENAME}"
          DESTINATION ${INSTALL_CMAKE_DIR})

  configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake"
    @ONLY IMMEDIATE)

  if (NOT TARGET uninstall)
    add_custom_target(uninstall
      "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake")
  else ()
    add_custom_target(${PROJECT_NAME}_uninstall
      "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake")
    add_dependencies(uninstall ${PROJECT_NAME}_uninstall)
  endif ()
endmacro()


