cmake_minimum_required(VERSION 3.0.2)

project(augfocus_openvr)

# -----------------------------------------------------------------------------
## SYSTEM PROPERTIES ##

# If not set, determines the running platform architecture.
if(NOT PLATFORM)
  if(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(PLATFORM 64)
  else()
    set(PLATFORM 32)
  endif()
endif()
message(STATUS "Compilation set for ${PLATFORM}bits architectures.")

set(SDL_REQUIRED_LIBRARIES SDL2)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  add_definitions(-DLINUX -DPOSIX)
  set(ARCH_TARGET linux64)

  if(${PLATFORM} MATCHES 32)
    message(WARNING "OpenVR x86 binaries not provided on GNU/Linux.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_MACOSX_RPATH 0)
  add_definitions(-DOSX -DPOSIX)
  set(ARCH_TARGET osx32)

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(SDL_REQUIRED_LIBRARIES ${SDL_REQUIRED_LIBRARIES} SDL2main)
  add_definitions(-D_WIN32)
  set(ARCH_TARGET win${PLATFORM})

  # Binaries path for thirdparties are not generics so we try to guess their suffixes.
  set(WINDOWS_PATH_SUFFIXES win${PLATFORM} Win${PLATFORM} x${PLATFORM})

  if(${PLATFORM} MATCHES 64)
    message(WARNING "SDL x64 runtime binaries not provided on Windows.")
  endif()
endif()

# -----------------------------------------------------------------------------
## PATHS ##

set(LIBRARIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libraries)
set(SHARED_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shared)

# Check that the steamVR SDK is installed
# (needed to prevent a segfault in OpenVR).
if(CMAKE_HOST_UNIX)
  find_file(OPENVRPATHS openvrpaths.vrpath PATHS $ENV{HOME}/.config/openvr "$ENV{HOME}/Library/Application Support/OpenVR/.openvr")
  if(${OPENVRPATHS} MATCHES OPENVRPATHS-NOTFOUND)
    message(FATAL_ERROR "${OPENVRPATHS} Please install SteamVR SDK to continue..")
  endif()
endif()

# Default output directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/${ARCH_TARGET})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/${ARCH_TARGET})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/${ARCH_TARGET})

# Force output directory destination, especially for MSVC (@so7747857).
function(setTargetOutputDirectory target)
  foreach(type RUNTIME LIBRARY ARCHIVE)
    set_target_properties(${target} PROPERTIES
      ${type}_OUTPUT_DIRECTORY         ${CMAKE_${type}_OUTPUT_DIRECTORY}
      ${type}_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_${type}_OUTPUT_DIRECTORY}
      ${type}_OUTPUT_DIRECTORY_RELEASE ${CMAKE_${type}_OUTPUT_DIRECTORY}
    )
  endforeach()
endfunction()

# -----------------------------------------------------------------------------
## COMPILER DETECTION ##
if(   (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
   OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
  # Better to use the prebuilt GNU preprocessor define __GNUC__,
  # kept for legacy reason with the sample code.
  add_definitions(-DGNUC)

  set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

  # Handles x86 compilation support on x64 arch.
  if(${PLATFORM} MATCHES 32)
    set(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS} -m32")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} /W2 /DEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MP /INCREMENTAL:NO")
else()
  message(FATAL_ERROR "Unsupported compiler '${CMAKE_CXX_COMPILER_ID}'")
endif()

# -----------------------------------------------------------------------------
## LIBRARIES ##

## OpenGL / GLU
find_package(OpenGL REQUIRED)

## OpenCV
find_package(OpenCV REQUIRED)

## zeroMQ in C++
find_package(cppzmq REQUIRED)

# msgpack
find_package(msgpack REQUIRED)

## GLEW 1.11
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # On GNU/Linux, glew can be found on the package manager.
  find_package(GLEW 1.11 REQUIRED)
else()
  # Otherwise, use the shipped binaries.
  find_library(GLEW_LIBRARIES
    NAMES
      GLEW
      glew32
    PATHS
      ${LIBRARIES_DIR}/glew/glew-1.11.0/lib/Release
    PATH_SUFFIXES
      osx32
      ${WINDOWS_PATH_SUFFIXES}
  )
  set(GLEW_INCLUDE_DIR ${LIBRARIES_DIR}/glew/glew-1.11.0/include)
endif()

## SDL 2
## osx32 and linux32 cmake can spot each others' .so and .a files, so keep them apart
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  foreach(lib ${SDL_REQUIRED_LIBRARIES})
    find_library(${lib}_LIBRARY
      NAMES
        ${lib}
      PATHS
        ${LIBRARIES_DIR}/sdl2-2.0.3/bin
      PATH_SUFFIXES
        osx32
    )
    list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
  endforeach()
else()
  foreach(lib ${SDL_REQUIRED_LIBRARIES})
    find_library(${lib}_LIBRARY
      NAMES
        ${lib}
      PATHS
        ${LIBRARIES_DIR}/sdl2-2.0.3/bin
      PATH_SUFFIXES
        linux${PLATFORM}
        ${WINDOWS_PATH_SUFFIXES}
    )
    list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
  endforeach()
endif()
set(SDL2_INCLUDE_DIR ${LIBRARIES_DIR}/sdl2-2.0.3/include)

## Vulkan
if(NOT (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
  find_library(VULKAN_LIBRARY
      NAMES
        vulkan-1
        vulkan
      PATHS
        ${LIBRARIES_DIR}/vulkan-1.0.49.0/lib
      PATH_SUFFIXES
        linux${PLATFORM}
        ${WINDOWS_PATH_SUFFIXES}
      NO_DEFAULT_PATH
  )
  set(VULKAN_INCLUDE_DIR ${LIBRARIES_DIR}/vulkan-1.0.49.0/include)
endif()

## OpenVR API path
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${LIBRARIES_DIR}/openvr-1.9.16/bin
    ${LIBRARIES_DIR}/openvr-1.9.16/lib
  PATH_SUFFIXES
    osx32
    linux64
    ${WINDOWS_PATH_SUFFIXES}
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${LIBRARIES_DIR}/openvr-1.9.16/include)


# -----------------------------------------------------------------------------
## SHARED SOURCES ##

file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)

include_directories(
  .
  ${OPENGL_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIR}
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
  ${cppzmq_INCLUDE_DIR}
  ${msgpackc_INCLUDE_DIRS}
)

if(NOT (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
  include_directories(${VULKAN_INCLUDE_DIR})
endif()

# -----------------------------------------------------------------------------
## SUBDIRECTORIES ##

add_subdirectory(src)

# -----------------------------------------------------------------------------
