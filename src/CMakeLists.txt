SET( CMAKE_BUILD_TYPE Release )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
MESSAGE(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

find_package(Armadillo 5.4 REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
#################################
#      Set Libraries
#################################
set(LIBRARIES armadillo)

# set(Eigen_INCLUDE_DIRS /usr/include/eigen3)
set(QP_DIRS ${SRC_PATH}/QP)
set(MPC_DIRS ${SRC_PATH}/MPC)
set(RTCONTROL_DIRS ${SRC_PATH}/RTControl)
set(KMP_DIRS ${SRC_PATH}/KMP)
set(Utils_DIRS ${SRC_PATH}/utils)


INCLUDE_DIRECTORIES(
	${SRC_PATH}
	${Utils_DIRS}
)

FILE(GLOB SOURCES
	${SRC_PATH}/main_function.cpp
	${Utils_DIRS}/EiQuadProg/*.cpp
	${QP_DIRS}/*.cpp
	${MPC_DIRS}/MPCClass.cpp
        ${RTCONTROL_DIRS}/MpcRTControlClass.cpp
        ${KMP_DIRS}/*.cpp
)


include_directories(include)

add_executable(${PROJ_NAME}.exe ${SOURCES})
target_link_libraries(${PROJ_NAME}.exe ${LIBRARIES})



