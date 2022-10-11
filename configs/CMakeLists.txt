aux_source_directory(. CONFIGS_SRC)

add_library(configs STATIC ${CONFIGS_SRC})

target_include_directories(configs PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
)