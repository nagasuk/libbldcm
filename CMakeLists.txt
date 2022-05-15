cmake_minimum_required(VERSION 3.8)

# [ Project Setting ]
## Common
project(libbldcm CXX)

## Options
option(LIBBLDCM_BUILD_SHARED_LIBS "Build libbldcm as a shared library" ON)

# [ For building this projects ]
if (LIBBLDCM_BUILD_SHARED_LIBS)
	add_library(bldcm SHARED)
else()
	add_library(bldcm STATIC)
endif()

target_sources(bldcm PRIVATE libbldcm.cpp register_map.cpp)
set_target_properties(bldcm PROPERTIES VERSION "1.0.0" SOVERSION "1")
target_include_directories(bldcm PUBLIC include)
target_compile_options(bldcm PRIVATE -Wall)
target_compile_features(bldcm PRIVATE cxx_std_17)

# [ Installation ]
install(TARGETS bldcm LIBRARY DESTINATION ${CMAKE_INSTALL_LIBIDR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include)

