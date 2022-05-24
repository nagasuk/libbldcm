cmake_minimum_required(VERSION 3.8)

# [ Project Setting ]
## Common
project(libbldcm VERSION 1.0.0 LANGUAGES CXX)

## Options
option(LIBBLDCM_BUILD_SHARED_LIBS "Build libbldcm as a shared library" ON)

## Find the package depended on by this library.
find_package(fpgasoc 1.0.1)

# [ For building this projects ]
if (LIBBLDCM_BUILD_SHARED_LIBS)
	add_library(bldcm SHARED)
else()
	add_library(bldcm STATIC)
endif()

## Register alias of namespace
add_library(bldcm::bldcm ALIAS bldcm)

## Some setting.
target_sources(bldcm PRIVATE
	libbldcm.cpp
	register_map.cpp
)
set_target_properties(bldcm PROPERTIES
	VERSION   "1.0.0"
	SOVERSION "1"
)
target_include_directories(bldcm PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(bldcm INTERFACE $<INSTALL_INTERFACE:include>)
target_link_libraries(bldcm PUBLIC fpgasoc)
target_compile_options(bldcm PRIVATE -Wall)
target_compile_features(bldcm PRIVATE cxx_std_17)

# [ Installation ]
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/bldcm-config-version.cmake
	COMPATIBILITY SameMajorVersion
)

include(GNUInstallDirs)
install(TARGETS bldcm 
	EXPORT bldcm-config
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} # for shared lib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} # for static lib
)

install(EXPORT bldcm-config
	NAMESPACE   bldcm::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bldcm
	EXPORT_LINK_INTERFACE_LIBRARIES
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/bldcm-config-version.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bldcm
)

