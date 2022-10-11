aux_source_directory(. SMOOTHER_SRC)

#link_directories的位置得放到add_executable前面
link_directories(
    "usr/local/lib"
    "usr/local/lib64"
    "usr/lib"
    ${PROJECT_BINARY_DIR}/lib
)

add_library(trajectory_smoother ${SMOOTHER_SRC})



find_package(Eigen3)

target_include_directories(trajectory_smoother PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
)


target_link_libraries(trajectory_smoother PUBLIC
    configs 
    common_math
    Eigen3::Eigen
    libipopt.so
    /usr/local/lib64/libadolc.so.2
)
