cmake_minimum_required( VERSION 2.8 )
project( eigen_ceres_tutorials )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

set(CMAKE_PREFIX_PATH "/usr/local/opencv3.2/share/OpenCV")
FIND_PACKAGE(OpenCV 3.2 REQUIRED)


add_executable( test1 test1.cpp )
add_executable( test2 test2.cpp )

add_executable( ceres_hello_world ceres_hello_world.cpp )
# 与Ceres链接
target_link_libraries( ceres_hello_world ${CERES_LIBRARIES})

add_executable( ceres_Powell_Function ceres_Powell_Function.cpp )
target_link_libraries( ceres_Powell_Function ${CERES_LIBRARIES})

add_executable( ceres_Curve_Fitting ceres_Curve_Fitting.cpp )
target_link_libraries( ceres_Curve_Fitting ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable( ceres_BA ceres_BA.cpp )
target_link_libraries( ceres_BA ${CERES_LIBRARIES} )
