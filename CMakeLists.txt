cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(masfm)

file(GLOB_RECURSE MASFM_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h*")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/")

######################################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/)
include(${CMAKE_SOURCE_DIR}/cmake/Specify3rdparty.cmake)

IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

if(NOT CMAKE_INSTALL_PREFIX)
	set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/matlab/ CACHE PATH ${CMAKE_SOURCE_DIR}/matlab/)
endif()

####### CV2CG
Specify3rdparty(
	CV2CG
	${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cv2cg/include
	"include directory for cv2cg"
	apriltag/apriltag.hpp
)
SET(LCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cv2cg/3rdparty/lch/include)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CV2CG_INCLUDE_DIR}/../cmake/)
include(${CV2CG_INCLUDE_DIR}/../cmake/SetCV2CGDependencies.cmake)
include(${CV2CG_INCLUDE_DIR}/../cmake/AutoAppConfig.cmake)
include(${CV2CG_INCLUDE_DIR}/../cmake/AutoAddSubDir.cmake)
include(${CV2CG_INCLUDE_DIR}/../cmake/BuildLCMType.cmake)
ADD_SUBDIRECTORY(${CV2CG_INCLUDE_DIR}/../application)
MESSAGE(STATUS "CV2CG_3RDPARTY_LINKEROPTION: " ${CV2CG_3RDPARTY_LINKEROPTION})

####### eigen
Specify3rdparty(
	EIGEN
	${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eigen/
	"include directory for Eigen"
	Eigen/Eigen
)

####### Ceres
find_package(Ceres REQUIRED)
MESSAGE(STATUS "CERES_INCLUDE_DIR: " ${Ceres_DIR}/../include)
MESSAGE(STATUS "CERES_LIBRARIES: " ${CERES_LIBRARIES})
MESSAGE(STATUS "GFLAGS_INCLUDE_DIRS: " ${GFLAGS_INCLUDE_DIRS})
MESSAGE(STATUS "GLOG_INCLUDE_DIRS: " ${GLOG_INCLUDE_DIRS})
include_directories(${Ceres_DIR}/../include)
include_directories(${GFLAGS_INCLUDE_DIRS})
include_directories(${glog_DIR}/../../../include)

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

target_link_libraries(masfm cmgraph ${CV2CG_3RDPARTY_LINKEROPTION} ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

set_target_properties(${ProjectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin")

INSTALL(TARGETS masfm DESTINATION bin)
INSTALL(DIRECTORY data/ DESTINATION data)
INSTALL(FILES source/masfm.ini data/masfm.in source/masfm.cam.ini DESTINATION bin)
INSTALL(TARGETS cmgraph DESTINATION lib)