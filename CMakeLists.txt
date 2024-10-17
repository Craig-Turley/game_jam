cmake_minimum_required(VERSION 3.30)

# Set CMake policies to NEW to handle warnings
cmake_policy(SET CMP0135 NEW)
cmake_policy(SET CMP0169 NEW)

project(mygame)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect Emscripten
if(CMAKE_SYSTEM_NAME MATCHES "Emscripten")
    set(EMSCRIPTEN TRUE)
endif()

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Link Cute Framework statically
set(CUTE_FRAMEWORK_STATIC ON)
set(CF_FRAMEWORK_BUILD_SAMPLES OFF)
set(CF_FRAMEWORK_BUILD_TESTS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include FetchContent module
include(FetchContent)

# Declare Cute Framework
FetchContent_Declare(
    cute
    GIT_REPOSITORY https://github.com/RandyGaul/cute_framework
    GIT_TAG 1.0.1  # Consider specifying a stable release tag
)

# Optionally, set Cute Framework to use SDL2 if such an option exists
# set(CUTE_FRAMEWORK_USE_SDL2 ON)  # Uncomment if applicable

# Make Cute Framework available
FetchContent_MakeAvailable(cute)

# Source code for your game
file(GLOB mygame_src CONFIGURE_DEPENDS
    "src/*.h"
    "src/*.c"
    "src/*.cpp"
)

add_executable(
    mygame
    ${mygame_src}
)

# Include directories
target_include_directories(mygame PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

# Link against Cute Framework (which should already link SDL2)
target_link_libraries(mygame PRIVATE cute)

# Support for web builds through Emscripten
if(EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    target_compile_options(mygame PUBLIC -O1 -fno-rtti -fno-exceptions)
    target_link_options(mygame PRIVATE -o mygame.html --preload-file ${CMAKE_SOURCE_DIR}/content@/content --emrun -s ASYNCIFY=1 -O1)
endif()

# macOS bundle settings
if(APPLE)
    set_target_properties(mygame PROPERTIES
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.myteam.mygame"
        MACOSX_BUNDLE_BUNDLE_VERSION "1.0.0"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0.0"
    )
endif()

# Windows-specific settings
if (MSVC)
    set_property(TARGET mygame PROPERTY VS_DEBUGGER_WORKING_DIRECTORY $<TARGET_FILE_DIR:mygame>)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT mygame)
endif()
