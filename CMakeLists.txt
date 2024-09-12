cmake_minimum_required(VERSION 3.16.3)
project(libserial_connect CXX)

set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

include_directories(
	include
)

link_directories(
	include
)

add_library(serial_connect SHARED
	src/serial_connect.cpp
)

set_target_properties(serial_connect PROPERTIES
	PUBLIC_HEADER include/serial_connect/serial_connect.hpp
)

install(TARGETS serial_connect
	EXPORT serial_connect-export
	LIBRARY DESTINATION lib
	INCLUDES DESTINATION include
	PUBLIC_HEADER DESTINATION include
)

install(EXPORT serial_connect-export
	FILE serial_connect-config.cmake
	DESTINATION share/cmake/serial_connect/
	EXPORT_LINK_INTERFACE_LIBRARIES
)

export(EXPORT serial_connect-export
	FILE serial_connect-export.cmake
)
