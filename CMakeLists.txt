CMAKE_MINIMUM_REQUIRED(VERSION 2.6) #设定版本
PROJECT(IR_sim_test) #设定工程名

SET( CMAKE_CXX_COMPILER "g++") #设定编译器
#SET(CMAKE_CXX_FLAGS "-msse3")
#SET(CMAKE_BUILD_TYPE Debug)
SET( CMAKE_BUILD_TYPE "Debug" )
SET( CMAKE_CXX_FLAGS "-std=c++11 -msse3" )

if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
   add_definitions( -O1 -DDEBUG -g -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 )
elseif( CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
   add_definitions( -O3 -DDEBUG -march=native -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 )
elseif( CMAKE_BUILD_TYPE STREQUAL "Release" )
   add_definitions( -O3 -march=native -msse -msse2 -mssse3 -msse4.1 -msse4.2 -ffast-math -ftree-loop-if-convert -funroll-loops -mfpmath=sse )
endif()

SET(RUN_PATH ./main)

FIND_PACKAGE(OpenCV REQUIRED )
FIND_PACKAGE(OpenGL REQUIRED )
FIND_PACKAGE(GLUT REQUIRED)

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
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#add cmake script files
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_work")

FIND_PACKAGE(Config++ REQUIRED)
INCLUDE_DIRECTORIES(${LIBCONFIG_INCLUDE_DIR})

include_directories(${IR_sim_SOURCE_DIR}/src)

aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/libelas ELAS_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/ADCensusBM AD_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SPS SPS_SRC_LIST)
#aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/STCA STCA_SRC_LIST)
#SSCA源码

aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SSCA SSCA_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SSCA/CC SSCA_CC_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SSCA/CANLC SSCA_NL_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SSCA/CAFilter SSCA_CF_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SSCA/CAST SSCA_ST_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SSCA/PPSG SSCA_SG_SRC_LIST)
aux_source_directory(/home/zhu/coding/IR_sim_test/src/stereo/reconstruct/SSCA/PPWM SSCA_WM_SRC_LIST)

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
  #${STCA_SRC_LIST}
  #SSCA文件源码
  ${SSCA_SRC_LIST}
  ${SSCA_CC_SRC_LIST}
  ${SSCA_CF_SRC_LIST}
  ${SSCA_NL_SRC_LIST}
  ${SSCA_ST_SRC_LIST}
  ${SSCA_SG_SRC_LIST}
  ${SSCA_WM_SRC_LIST}
)

target_link_libraries( main ${OpenCV_LIBRARIES} )
target_link_libraries( main "/usr/local/lib/libftd2xx.so" )
target_link_libraries(main pthread)
target_link_libraries( main ${OPENGL_LIBRARIES} )
target_link_libraries( main ${GLUT_LIBRARIES} )
target_link_libraries( main ${PCL_LIBRARIES} )
target_link_libraries(main  ${LIBCONFIG_LIBRARIES})
