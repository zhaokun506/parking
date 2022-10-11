aux_source_directory(. MATH_SRC)

add_library(common_math ${MATH_SRC})

message(工程目录是：${PROJECT_SOURCE_DIR})
find_package(absl)
target_include_directories(common_math PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
)

target_link_libraries(common_math PUBLIC
               absl::strings
)