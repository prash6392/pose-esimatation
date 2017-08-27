cmake_minimum_required(VERSION 3.2)

project(PoseEstimation)

# include common cmake search paths
include(${CMAKE_CURRENT_SOURCE_DIR}/../../CmakeCommon.txt)

# set sources
file (GLOB SOURCES
src/*.cpp
src/*.h
${PUGIXML_FILE})

add_executable(${PROJECT_NAME} ${SOURCES})

# set include directories
target_include_directories(${PROJECT_NAME}
PUBLIC
${EIGEN_INCLUDE}
${CERES_SOLVER_INCLUDE}
${DLIB_INCLUDE}
${PUGIXML_INCLUDE})

# set target link libraries
find_package(Threads REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
target_link_libraries(${PROJECT_NAME}
    ${CERES_LIB_STATIC}
    ${DLIB_LIB_STATIC}
    ${GLOG_LIB}
    pthread
    ${SUITESPARSE}
    ${BLAS_LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
