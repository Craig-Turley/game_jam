# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /System/Volumes/Data/Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /System/Volumes/Data/Applications/CMake.app/Contents/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/craig/Code/game_jam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/craig/Code/game_jam

# Utility rule file for uninstall.

# Include any custom commands dependencies for this target.
include _deps/sdl2-build/CMakeFiles/uninstall.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/sdl2-build/CMakeFiles/uninstall.dir/progress.make

_deps/sdl2-build/CMakeFiles/uninstall:
	cd /Users/craig/Code/game_jam/_deps/sdl2-build && /System/Volumes/Data/Applications/CMake.app/Contents/bin/cmake -P /Users/craig/Code/game_jam/_deps/sdl2-build/cmake_uninstall.cmake

uninstall: _deps/sdl2-build/CMakeFiles/uninstall
uninstall: _deps/sdl2-build/CMakeFiles/uninstall.dir/build.make
.PHONY : uninstall

# Rule to build all files generated by this target.
_deps/sdl2-build/CMakeFiles/uninstall.dir/build: uninstall
.PHONY : _deps/sdl2-build/CMakeFiles/uninstall.dir/build

_deps/sdl2-build/CMakeFiles/uninstall.dir/clean:
	cd /Users/craig/Code/game_jam/_deps/sdl2-build && $(CMAKE_COMMAND) -P CMakeFiles/uninstall.dir/cmake_clean.cmake
.PHONY : _deps/sdl2-build/CMakeFiles/uninstall.dir/clean

_deps/sdl2-build/CMakeFiles/uninstall.dir/depend:
	cd /Users/craig/Code/game_jam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/craig/Code/game_jam /Users/craig/Code/game_jam/_deps/sdl2-src /Users/craig/Code/game_jam /Users/craig/Code/game_jam/_deps/sdl2-build /Users/craig/Code/game_jam/_deps/sdl2-build/CMakeFiles/uninstall.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : _deps/sdl2-build/CMakeFiles/uninstall.dir/depend

