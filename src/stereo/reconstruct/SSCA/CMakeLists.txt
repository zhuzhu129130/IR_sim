cmake_minimum_required(VERSION 2.6)
project(SSCA)

find_package(OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIR})

link_directories(${OpenCV_LIBRARY_DIR})

set(CMAKE_BUILD_TYPE Debug)

add_executable(main
./main.cpp
./SSCA.cpp

./CAFilter/BFCA.cpp
./CAFilter/BilateralFilter.cpp
./CAFilter/BoxCA.cpp
./CAFilter/GFCA.cpp
./CAFilter/GuidedFilter.cpp

./CANLC/ctmf.c
./CANLC/NLCCA.cpp
./CANLC/qx_basic.cpp
./CANLC/qx_mst_kruskals_image.cpp
./CANLC/qx_nonlocal_cost_aggregation.cpp
./CANLC/qx_tree_filter.cpp

#./CAST/ctmf.c
./CAST/SegmentTree.cpp
./CAST/STCA.cpp
./CAST/StereoDisparity.cpp
./CAST/StereoHelper.cpp
./CAST/Toolkit.cpp

./CC/CenCC.cpp
./CC/CGCC.cpp
./CC/GrdCC.cpp

./PPSG/SGPP.cpp

./PPWM/WMPP.cpp
)

target_link_libraries( main ${OpenCV_LIBRARIES} )











