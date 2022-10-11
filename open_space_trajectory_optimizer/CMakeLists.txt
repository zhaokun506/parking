add_library(open_space_trajectory_optimizer STATIC open_space_trajectory_optimizer.cc)

target_include_directories(open_space_trajectory_optimizer PUBLIC
                        ${PROJECT_SOURCE_DIR}#工程目录
                         "usr/include"#系统目录
                         "usr/local/include"#安装文件目录
                         "/usr/include/python3.6m"#python目录
)

target_link_libraries(open_space_trajectory_optimizer PUBLIC
                    common_math 
                    configs 
                    coarse_trajectory_generator
                    trajectory_smoother
)