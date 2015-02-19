CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(bullet_narrowphase)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")


FIND_PACKAGE(ChronoEngine)
include_directories(${CHRONOENGINE_INCLUDES})

ADD_EXECUTABLE(bullet_narrowphase	main.cpp narrowphase/gjk_epa.cpp narrowphase/voronoi_simplex.cpp)
TARGET_LINK_LIBRARIES(bullet_narrowphase ${CHRONOENGINE_LIBRARIES} )