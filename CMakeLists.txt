CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

SET(PROJ_NAME MPC_WALK)
# set(TARGET ${PROJ_NAME})

SET(CMAKE_BUILD_TYPE Release)
 
PROJECT(${PROJ_NAME} C CXX)
MESSAGE(STATUS "Project: ${PROJ_NAME}")    
MESSAGE(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")


FIND_PACKAGE ( Threads REQUIRED )
SET(SUB_SRC  src)

SET(SRC_PATH  ${PROJECT_SOURCE_DIR}/${SUB_SRC})
ADD_SUBDIRECTORY(${SUB_SRC})

