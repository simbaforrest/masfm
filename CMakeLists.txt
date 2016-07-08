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
ADD_DEFINITIONS(-DOPENCV_FOUND)
MESSAGE(STATUS "CV2CG_3RDPARTY_LINKEROPTION: " ${CV2CG_3RDPARTY_LINKEROPTION})

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
Specify3rdpartyLib(
	LIBGLOG_LIB_RELEASE
	c:/glog/lib/Release/libglog.lib
	"release lib for libglog"
	true
)
Specify3rdpartyLib(
	LIBGLOG_LIB_DEBUG
	c:/glog/lib/Debug/libglog.lib
	"debug lib for libglog"
	false
)
# use static or dynamic libglog
option(OPT_USE_LIBGLOG_STATIC "use libglog_static.lib instead of libglog.lib" ON)
if(OPT_USE_LIBGLOG_STATIC)
	add_definitions( -DLIBGLOG_IS_STATIC )
endif()

####### ceres
Specify3rdparty(
	CERES
	c:/ceres/include
	"include directory for ceres"
	ceres/ceres.h
)
include_directories("${CERES_INCLUDE_DIR}/../../win/include/")
Specify3rdpartyLib(
	CERES_LIB_RELEASE
	c:/ceres/lib/Release/ceres.lib
	"release lib for ceres"
	true
)
Specify3rdpartyLib(
	CERES_LIB_DEBUG
	c:/glog/lib/Debug/ceres.lib
	"debug lib for ceres"
	false
)
# use static or dynamic ceres
option(OPT_USE_CERES_STATIC "use ceres_static.lib instead of ceres.lib" ON)
if(OPT_USE_CERES_STATIC)
	add_definitions( -DCERES_USING_STATIC_LIBRARY )
	
	IF(WIN32)
		set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /GL /Ot" )
		set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /LTCG" )
		set( CMAKE_STATIC_LINKER_FLAGS  "${CMAKE_STATIC_LINKER_FLAGS} /LTCG" )
	ENDIF()
endif()

######################################################################################
add_library(cmgraph STATIC
	source/cmgraph.cpp
	${MASFM_HEADERS}
)

add_executable(masfm
	source/masfm.cpp
	${MASFM_HEADERS}
	source/masfm.ini
)

target_link_libraries(masfm cmgraph ${CV2CG_3RDPARTY_LINKEROPTION})
target_link_libraries(masfm optimized ${LIBGLOG_LIB_RELEASE} ${CERES_LIB_RELEASE})
target_link_libraries(masfm debug ${LIBGLOG_LIB_DEBUG} ${CERES_LIB_DEBUG})

set_target_properties(${ProjectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin")

INSTALL(TARGETS masfm DESTINATION bin)
INSTALL(FILES source/masfm.ini data/masfm.in DESTINATION bin)
INSTALL(TARGETS cmgraph DESTINATION lib)