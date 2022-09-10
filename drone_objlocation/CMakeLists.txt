cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME drone_objlocation)
project(${PROJECT_NAME} VERSION 1.0)

# 添加头文件
include_directories(
    include
    "/usr/include/eigen3"
)
# 添加源文件
aux_source_directory(./src DIR_SRCS)
### 生成lib
add_library(DroneObjlocation SHARED ${DIR_SRCS})

### 安装
# # 自定义安装路径
# set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)
# message(STATUS "Install path="${CMAKE_INSTALL_PREFIX})

# 写入库的版本信息
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        DroneObjlocationConfigVersion.cmake
        VERSION ${PACKAGE_VERSION}
        COMPATIBILITY AnyNewerVersion
)

# 将库文件，可执行文件安装到指定目录
install(TARGETS DroneObjlocation
    EXPORT DroneObjlocationTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# 拷贝头文件
install(DIRECTORY include/
        DESTINATION include/geolocation
)

# 将库的相关信息输出为DroneObjlocationConfig.cmake，供find_package查找
install(EXPORT DroneObjlocationTargets
        FILE DroneObjlocationTargets.cmake
        DESTINATION lib/cmake/DroneObjlocation
        )

configure_file(DroneObjlocationConfig.cmake.in DroneObjlocationConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/DroneObjlocationConfig.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/DroneObjlocationConfigVersion.cmake"
        DESTINATION lib/cmake/DroneObjlocation
)

