# This macro allows user to specify 3rdparty libs
#	simbaforrest

MACRO(Specify3rdparty name default_path explain_text check_file_name)

if(NOT ${name}_INCLUDE_DIR)
	set(${name}_INCLUDE_DIR "${default_path}" CACHE PATH ${explain_text})
endif()
if(EXISTS ${${name}_INCLUDE_DIR}/${check_file_name})
	include_directories(${${name}_INCLUDE_DIR})
	message(STATUS "${name}: ${${name}_INCLUDE_DIR}")
else()
	message(FATAL_ERROR "Please specify a valid root directory for ${name}!")
endif()

ENDMACRO()



MACRO(Specify3rdpartyLib name default_path explain_text is_required)

if(NOT ${name})
	set(${name} "${default_path}" CACHE FILEPATH ${explain_text})
endif()
if(EXISTS ${${name}})
	message(STATUS "${name}: ${${name}}")
else()
	IF(is_required)
		message(FATAL_ERROR "Please specify a valid lib path for ${name}!")
	ELSE()
		message(WARNING "lib path for ${name} not found, ignored!")
	ENDIF()
endif()

ENDMACRO()