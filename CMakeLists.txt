cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(masfm)

file(GLOB_RECURSE MASFM_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h*")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/")

######################################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/)
include(${CMAKE_SOURCE_DIR}/cmake/Specify3rdparty.cmake)

####### LCH
Specify3rdparty(
	LCH
	${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lch/include
	"include directory for LCH"
	lch.hpp
)

####### CV2CG
Specify3rdparty(
	CV2CG
	${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cv2cg/include
	"include directory for cv2cg"
	apriltag/apriltag.hpp
)
include(${CV2CG_INCLUDE_DIR}/../cmake/SetCV2CGDependencies.cmake)

####### eigen
Specify3rdparty(
	EIGEN
	${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eigen/
	"include directory for Eigen"
	Eigen/Eigen
)

####### libglog
Specify3rdparty(
	LIBGLOG
	c:/glog/include
	"include directory for libglog"
	glog/logging.h
)
# libglog lib
if(NOT LIBGLOG_LIB_DIR)
	set(LIBGLOG_LIB_DIR "c:/glog/lib" CACHE PATH "lib directory for libglog")
endif()
if(EXISTS ${LIBGLOG_LIB_DIR}/libglog.lib OR EXISTS ${LIBGLOG_LIB_DIR}/libglog_static.lib)
	link_directories(${LIBGLOG_LIB_DIR})
	message(STATUS "libglog lib: ${LIBGLOG_LIB_DIR}")
else()
	message(FATAL_ERROR "Please specify a valid lib directory for libglog!")
endif()
# use static or dynamic libglog
option(OPT_USE_LIBGLOG_STATIC "use libglog_static.lib instead of libglog.lib" ON)
if(OPT_USE_LIBGLOG_STATIC)
	add_definitions( -DLIBGLOG_IS_STATIC )
	set(LIBGLOG_LIB libglog_static.lib)
else()
	set(LIBGLOG_LIB libglog.lib)
endif()
message(STATUS "LIBGLOG_LIB=${LIBGLOG_LIB}")

####### ceres
Specify3rdparty(
	CERES
	c:/ceres/include
	"include directory for ceres"
	ceres/ceres.h
)
include_directories("${CERES_INCLUDE_DIR}/../../win/include/")
# ceres lib
if(NOT CERES_LIB_DIR)
	set(CERES_LIB_DIR "c:/ceres/lib" CACHE PATH "lib directory for ceres")
endif()
if(EXISTS ${CERES_LIB_DIR}/ceres.lib OR EXISTS ${CERES_LIB_DIR}/ceres_static.lib)
	link_directories(${CERES_LIB_DIR})
	message(STATUS "ceres lib: ${CERES_LIB_DIR}")
else()
	message(FATAL_ERROR "Please specify a valid lib directory for ceres!")
endif()
# use static or dynamic ceres
option(OPT_USE_CERES_STATIC "use ceres_static.lib instead of ceres.lib" ON)
if(OPT_USE_CERES_STATIC)
	set(CERES_LIB ceres_static.lib)
else()
	set(CERES_LIB ceres.lib)
endif()
message(STATUS "CERES_LIB=${CERES_LIB}")

######################################################################################
add_library(cmgraph STATIC
	source/cmgraph.cpp
	${MASFM_HEADERS}
)

add_executable(masfm
	source/masfm.cpp
	${MASFM_HEADERS}
	source/masfm.cfg
)

target_link_libraries(masfm cmgraph ${CERES_LIB} ${LIBGLOG_LIB})
set_target_properties(${ProjectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin")

