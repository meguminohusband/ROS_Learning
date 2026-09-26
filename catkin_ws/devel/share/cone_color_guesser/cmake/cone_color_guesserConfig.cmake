# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(cone_color_guesser_CONFIG_INCLUDED)
  return()
endif()
set(cone_color_guesser_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(cone_color_guesser_SOURCE_PREFIX /Users/nathaniel/Documents/Python_Project/ROS/catkin_ws/src/cone_color_guesser)
  set(cone_color_guesser_DEVEL_PREFIX /Users/nathaniel/Documents/Python_Project/ROS/catkin_ws/devel)
  set(cone_color_guesser_INSTALL_PREFIX "")
  set(cone_color_guesser_PREFIX ${cone_color_guesser_DEVEL_PREFIX})
else()
  set(cone_color_guesser_SOURCE_PREFIX "")
  set(cone_color_guesser_DEVEL_PREFIX "")
  set(cone_color_guesser_INSTALL_PREFIX /Users/nathaniel/Documents/Python_Project/ROS/catkin_ws/install)
  set(cone_color_guesser_PREFIX ${cone_color_guesser_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'cone_color_guesser' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(cone_color_guesser_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/Users/nathaniel/Documents/Python_Project/ROS/catkin_ws/src/cone_color_guesser/include;/Users/nathaniel/micromamba/envs/ros_noetic/include/pcl-1.14;/Users/nathaniel/micromamba/envs/ros_noetic/include/eigen3;/Users/nathaniel/micromamba/envs/ros_noetic/include " STREQUAL " ")
  set(cone_color_guesser_INCLUDE_DIRS "")
  set(_include_dirs "/Users/nathaniel/Documents/Python_Project/ROS/catkin_ws/src/cone_color_guesser/include;/Users/nathaniel/micromamba/envs/ros_noetic/include/pcl-1.14;/Users/nathaniel/micromamba/envs/ros_noetic/include/eigen3;/Users/nathaniel/micromamba/envs/ros_noetic/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Nathaniel <202600407015@stumail.sztu.edu.cn>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${cone_color_guesser_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'cone_color_guesser' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'cone_color_guesser' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/Users/nathaniel/Documents/Python_Project/ROS/catkin_ws/src/cone_color_guesser/${idir}'.  ${_report}")
    endif()
    _list_append_unique(cone_color_guesser_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_common.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_sample_consensus.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_kdtree.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_octree.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_filters.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_features.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_ml.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_segmentation.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_search.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libpcl_io.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libboost_system.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libboost_iostreams.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libboost_filesystem.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkChartsCore-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkCommonColor-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkCommonComputationalGeometry-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkCommonCore-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkCommonDataModel-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkCommonExecutionModel-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkCommonMath-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkCommonMisc-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkCommonTransforms-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkFiltersCore-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkFiltersExtraction-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkFiltersGeneral-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkFiltersGeometry-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkFiltersModeling-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkFiltersSources-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkImagingCore-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkImagingSources-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkInteractionImage-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkInteractionStyle-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkInteractionWidgets-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkIOCore-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkIOGeometry-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkIOImage-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkIOLegacy-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkIOPLY-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkRenderingAnnotation-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkRenderingCore-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkRenderingContext2D-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkRenderingLOD-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkRenderingFreeType-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkViewsCore-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkViewsContext2D-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkRenderingOpenGL2-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkRenderingContextOpenGL2-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libvtkGUISupportQt-9.3.9.3.dylib;/Users/nathaniel/micromamba/envs/ros_noetic/lib/libflann_cpp.1.9.dylib")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general|rt|pthread|dl)$")
    list(APPEND cone_color_guesser_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND cone_color_guesser_LIBRARIES ${library})
  elseif(${library} MATCHES "^-framework")
    list(APPEND cone_color_guesser_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT cone_color_guesser_NUM_DUMMY_TARGETS)
      set(cone_color_guesser_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::cone_color_guesser::wrapped-linker-option${cone_color_guesser_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR cone_color_guesser_NUM_DUMMY_TARGETS "${cone_color_guesser_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::cone_color_guesser::wrapped-linker-option${cone_color_guesser_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND cone_color_guesser_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND cone_color_guesser_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND cone_color_guesser_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /Users/nathaniel/Documents/Python_Project/ROS/catkin_ws/devel/lib;/Users/nathaniel/micromamba/envs/ros_noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(cone_color_guesser_LIBRARY_DIRS ${lib_path})
      list(APPEND cone_color_guesser_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'cone_color_guesser'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND cone_color_guesser_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(cone_color_guesser_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${cone_color_guesser_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;sensor_msgs;pcl_ros;pcl_conversions;visualization_msgs;tf2_ros")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 cone_color_guesser_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${cone_color_guesser_dep}_FOUND)
      find_package(${cone_color_guesser_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${cone_color_guesser_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(cone_color_guesser_INCLUDE_DIRS ${${cone_color_guesser_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(cone_color_guesser_LIBRARIES ${cone_color_guesser_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${cone_color_guesser_dep}_LIBRARIES})
  _list_append_deduplicate(cone_color_guesser_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(cone_color_guesser_LIBRARIES ${cone_color_guesser_LIBRARIES})

  _list_append_unique(cone_color_guesser_LIBRARY_DIRS ${${cone_color_guesser_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(cone_color_guesser_EXPORTED_TARGETS ${${cone_color_guesser_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${cone_color_guesser_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
