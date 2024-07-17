# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/craig/Code/game_jam/_deps/cute-src")
  file(MAKE_DIRECTORY "/Users/craig/Code/game_jam/_deps/cute-src")
endif()
file(MAKE_DIRECTORY
  "/Users/craig/Code/game_jam/_deps/cute-build"
  "/Users/craig/Code/game_jam/_deps/cute-subbuild/cute-populate-prefix"
  "/Users/craig/Code/game_jam/_deps/cute-subbuild/cute-populate-prefix/tmp"
  "/Users/craig/Code/game_jam/_deps/cute-subbuild/cute-populate-prefix/src/cute-populate-stamp"
  "/Users/craig/Code/game_jam/_deps/cute-subbuild/cute-populate-prefix/src"
  "/Users/craig/Code/game_jam/_deps/cute-subbuild/cute-populate-prefix/src/cute-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/craig/Code/game_jam/_deps/cute-subbuild/cute-populate-prefix/src/cute-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/craig/Code/game_jam/_deps/cute-subbuild/cute-populate-prefix/src/cute-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
