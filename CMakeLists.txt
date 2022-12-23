CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

PROJECT(framecap)

FIND_PACKAGE(PkgConfig)
FIND_PACKAGE(freenect2 REQUIRED)

file(GLOB SOURCES
    header-folder/*.h
    source-folder/*.cpp
)

ADD_EXECUTABLE(framecap
  ${SOURCES}
)
