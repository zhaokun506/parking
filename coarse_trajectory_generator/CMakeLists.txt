aux_source_directory(. HYBRID_SRC)

add_library(coarse_trajectory_generator ${HYBRID_SRC})

target_include_directories(coarse_trajectory_generator PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
)


target_link_libraries(coarse_trajectory_generator PUBLIC 
                        common_math
                        configs
                        planning_data
)