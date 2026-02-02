cmake_minimum_required(VERSION 3.15)

project(PolarCodecNR
    VERSION 1.0.0
    DESCRIPTION "Polar Codec NR implementation"
    LANGUAGES C
)

# --------------------------------------------------
# Global settings
# --------------------------------------------------

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# --------------------------------------------------
# Sources & headers
# --------------------------------------------------

file(GLOB_RECURSE POLAR_SOURCES
    src/*.c
)

file(GLOB_RECURSE POLAR_HEADERS
    headers/*.h
)

# --------------------------------------------------
# Library target
# --------------------------------------------------

add_library(PolarCodecNR
    ${POLAR_SOURCES}
    ${POLAR_HEADERS}
)

# Public include directory
target_include_directories(PolarCodecNR
    PUBLIC
        ${PROJECT_SOURCE_DIR}/headers
)

# --------------------------------------------------
# Compiler warnings (portable)
# --------------------------------------------------

if(MSVC)
    target_compile_options(PolarCodecNR PRIVATE /W4)
else()
    target_compile_options(PolarCodecNR PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()

# --------------------------------------------------
# Platform-specific defines
# --------------------------------------------------

if(WIN32)
    target_compile_definitions(PolarCodecNR PRIVATE PLATFORM_WINDOWS)
elseif(APPLE)
    target_compile_definitions(PolarCodecNR PRIVATE PLATFORM_MACOS)
elseif(UNIX)
    target_compile_definitions(PolarCodecNR PRIVATE PLATFORM_LINUX)
endif()

# --------------------------------------------------
# Install rules (optional but universal)
# --------------------------------------------------

install(TARGETS PolarCodecNR
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY headers/
    DESTINATION include
)
