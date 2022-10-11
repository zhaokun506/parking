aux_source_directory(./path  PATH_SRC )
aux_source_directory(./speed SPEED_SRC )
aux_source_directory(./piecewise_jerk PIECE_SRC)
aux_source_directory(./trajectory SPEED_SRC )

add_library(planning_data ${PATH_SRC} ${SPEED_SRC} ${PIECE_SRC} ${SPEED_SRC})

find_package(osqp)

target_include_directories(planning_data PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
)

target_link_libraries(planning_data PUBLIC
                    common_math
                    osqp::osqp
)
