cmake_minimum_required(VERSION 3.5)
project(sls_qsf_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

# 生成代码和手写 C ABI 包装一起编成共享库，供 C++ 控制器和 Python ctypes 调用。
add_library(${PROJECT_NAME} SHARED
  src/qsf_c_api.cpp
  src/generated/QSFController.cpp
  src/generated/QSFGeometricController.cpp
  src/generated/QSFIntegralController.cpp
  src/generated/rtGetInf.cpp
  src/generated/rtGetNaN.cpp
  src/generated/rt_nonfinite.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    # include 是稳定 API，src/generated 只作为构建期内部头文件暴露。
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/generated>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(
  # 安装共享库后，Python wrapper 可在 ament prefix/lib 下找到 libsls_qsf_core.so。
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
