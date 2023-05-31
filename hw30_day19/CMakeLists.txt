cmake_minimum_required( VERSION 3.16 )
project( BA )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++17" )


find_package( OpenCV REQUIRED )


# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
#set( G2O_ROOT /usr/local/include/g2o )
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

#target_link_libraries(g2o_test ${G2O_LIBS})

# 添加头文件
include_directories( "/usr/include/eigen3"
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        )

add_executable( BA BA-3Dto2D.cpp )

target_link_libraries( BA
        ${OpenCV_LIBS}
        ${CSPARSE_LIBRARY}
        ${G2O_LIBS}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        )


