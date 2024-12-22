# 设置CMake的最低版本要求为3.8，确保项目使用兼容的CMake版本
cmake_minimum_required(VERSION 3.8)

# 定义项目名称为chapt4_interfaces
project(chapt4_interfaces)

# 如果编译器是GCC或Clang，添加编译选项以启用额外的警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
# 查找ament_cmake包，这是构建ROS 2包所必需的
find_package(ament_cmake REQUIRED)
# 查找rosidl_default_generators包，用于从接口定义生成代码
find_package(rosidl_default_generators REQUIRED)
# 查找sensor_msgs包，提供传感器的标准消息定义
find_package(sensor_msgs REQUIRED)

# 为项目中定义的接口生成代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FaceDetector.srv"
  "srv/Patrol.srv"
  DEPENDENCIES sensor_msgs
)

# 如果启用了测试构建，查找ament_lint_auto包进行代码检查
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 下面的行跳过了版权检查的linter
  # 当所有源文件添加了版权声明和许可证时，请取消注释该行
  set(ament_cmake_copyright_FOUND TRUE)
  # 下面的行跳过了cpplint（仅在git仓库中有效）
  # 当此包位于git仓库并且所有源文件添加了版权声明和许可证时，请取消注释该行
  set(ament_cmake_cpplint_FOUND TRUE)
  # 自动查找测试依赖项
  ament_lint_auto_find_test_dependencies()
endif()

# 使用ament准备包以供安装
ament_package()