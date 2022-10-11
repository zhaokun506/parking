# aux_source_directory(. MAP_SRC)

add_library(open_space_map open_space_map.cc)


target_include_directories(open_space_map PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
                             "/usr/include/python3.6m"#python目录
)


target_link_libraries(open_space_map PUBLIC
    common_math
    matplotlib_cpp
    planning_data
)
