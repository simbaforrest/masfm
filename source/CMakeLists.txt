cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(masfm)

file(GLOB_RECURSE MASFM_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/../include/*.h*)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/./")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include/")

# LCH
if(NOT LCH_INCLUDE_DIR)
	set(LCH_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lch/include" CACHE PATH "include directory for LCH")
endif()
if(EXISTS ${LCH_INCLUDE_DIR}/lch.hpp)
	include_directories(${LCH_INCLUDE_DIR})
	message(STATUS "LCH: ${LCH_INCLUDE_DIR}")
else()
	message(FATAL_ERROR "Please specify a valid root directory for LCH!")
endif()

# eigen
if(NOT EIGEN_INCLUDE_DIR)
	set(EIGEN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../eigen/" CACHE PATH "include directory for Eigen")
endif()
if(EXISTS ${EIGEN_INCLUDE_DIR}/Eigen/Eigen)
	include_directories(${EIGEN_INCLUDE_DIR})
	message(STATUS "Eigen: ${EIGEN_INCLUDE_DIR}")
else()
	message(FATAL_ERROR "Please specify a valid root directory for Eigen!")
endif()

# libglog include
if(NOT LIBGLOG_INCLUDE_DIR)
	set(LIBGLOG_INCLUDE_DIR "c:/glog/include" CACHE PATH "include directory for libglog")
endif()
if(EXISTS ${LIBGLOG_INCLUDE_DIR}/glog/logging.h)
	include_directories(${LIBGLOG_INCLUDE_DIR})
	message(STATUS "libglog: ${LIBGLOG_INCLUDE_DIR}")
else()
	message(FATAL_ERROR "Please specify a valid include directory for libglog!")
endif()
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
	set(LIBGLOG_LIB libglog_static.lib)
else()
	set(LIBGLOG_LIB libglog.lib)
endif()
message(STATUS "LIBGLOG_LIB=${LIBGLOG_LIB}")

# ceres include
if(NOT CERES_INCLUDE_DIR)
	set(CERES_INCLUDE_DIR "c:/ceres/include" CACHE PATH "include directory for ceres")
endif()
if(EXISTS ${CERES_INCLUDE_DIR}/ceres/ceres.h)
	include_directories(${CERES_INCLUDE_DIR})
	include_directories("${CERES_INCLUDE_DIR}/../../win/include/")
	message(STATUS "ceres: ${CERES_INCLUDE_DIR}")
else()
	message(FATAL_ERROR "Please specify a valid include directory for ceres!")
endif()
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

add_library(cmgraph STATIC
	cmgraph.cpp
	${MASFM_HEADERS}
	)

add_executable(masfm masfm.cpp ${MASFM_HEADERS} masfm.cfg)

target_link_libraries(masfm cmgraph ${CERES_LIB} ${LIBGLOG_LIB})
set_target_properties(${ProjectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin")

