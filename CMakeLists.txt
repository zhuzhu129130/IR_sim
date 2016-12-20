cmake_minimum_required(VERSION 2.6)
project(IR_sim_test)

#SET(CMAKE_CXX_FLAGS "-msse3")
#set(CMAKE_BUILD_TYPE Debug)
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -msse3" )

if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
   add_definitions( -O1 -DDEBUG -g -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 )
elseif( CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
   add_definitions( -O3 -DDEBUG -march=native -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 )
elseif( CMAKE_BUILD_TYPE STREQUAL "Release" )
   add_definitions( -O3 -march=native -msse -msse2 -mssse3 -msse4.1 -msse4.2 -ffast-math -ftree-loop-if-convert -funroll-loops -mfpmath=sse )
endif()


find_package(OpenCV REQUIRED )
find_package(OpenGL REQUIRED )
find_package(GLUT REQUIRED)

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )

include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${OpenGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIR})
link_directories(${OpenGL_LIBRARY_DIR})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#add cmake script files
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_work")

FIND_PACKAGE(Config++ REQUIRED)
INCLUDE_DIRECTORIES(${LIBCONFIG_INCLUDE_DIR})

include_directories(${IR_sim_SOURCE_DIR}/src)

aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/libelas ELAS_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/ADCensusBM AD_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SPS SPS_SRC_LIST)

add_executable(main
   ./src/main.cpp
   ./src/paraReader.cpp
   ./src/Variable.cpp 
   ./src/DVFileStore.cpp 
   ./src/IRImg_AR.cpp 
   ./src/Graphics_IRrad.cpp
   ./src/DataType.h
   ./src/Atmod.cpp
   ./src/readIR/ftcamif.cpp

  ./src/stereo/Pose2Dto2D.cpp
  ./src/stereo/basefunc/basefunc.cpp
  ./src/stereo/calib/stereocalib.cpp
  ./src/stereo/rectify/stereorectify.cpp
  ./src/stereo/reconstruct/stereoreconstruction.cpp
  ${ELAS_SRC_LIST}
  ${AD_SRC_LIST}
  ${SPS_SRC_LIST}
)

target_link_libraries( main ${OpenCV_LIBRARIES} )
target_link_libraries( main "/usr/local/lib/libftd2xx.so" )
target_link_libraries(main pthread)
target_link_libraries( main ${OPENGL_LIBRARIES} )
target_link_libraries( main ${GLUT_LIBRARIES} )
target_link_libraries( main ${PCL_LIBRARIES} )
target_link_libraries(main  ${LIBCONFIG_LIBRARIES})








