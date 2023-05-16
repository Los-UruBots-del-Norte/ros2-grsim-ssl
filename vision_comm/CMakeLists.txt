cmake_minimum_required(VERSION 3.5)
project(vision_comm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(krssg_ssl_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# Establecer la ruta de los archivos .proto

set(PROTOBUF_IMPORT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/vision_comm/proto")

#message(${PROTOBUF_IMPORT_DIRS})

# Generar los archivos .pb.h y .pb.cc a partir de los .proto
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS 
  ${PROTOBUF_IMPORT_DIRS}/messages_robocup_ssl_detection.proto
  ${PROTOBUF_IMPORT_DIRS}/messages_robocup_ssl_geometry.proto
  ${PROTOBUF_IMPORT_DIRS}/messages_robocup_ssl_wrapper.proto
  ${PROTOBUF_IMPORT_DIRS}/messages_robocup_ssl_refbox_log.proto
)

# Agregar ruta de búsqueda de archivos de cabecera de protobuf
include_directories(
  include
  ${PROTOBUF_IMPORT_DIRS}
  ${ament_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/${PROJECT_NAME}
)

# Agregar dependencias
add_executable(
  vision_comm 
  src/vision_node.cpp 
  src/netraw.cpp 
  src/robocup_ssl_client.cpp
)

# Especificar las dependencias del objetivo
ament_target_dependencies(
  vision_comm
  krssg_ssl_msgs
  rclcpp
  rosidl_default_runtime
)

# Add the include directories for your dependencies here
target_include_directories(vision_comm 
PRIVATE ${Protobuf_LIBRARIES}
)

# Add the generated source files to your target
target_sources(vision_comm 
PRIVATE ${PROTO_SRCS} ${PROTO_HDRS}
)

# Especificar las bibliotecas de las que depende el objetivo
target_link_libraries(
  vision_comm
  ${Protobuf_LIBRARIES} 
  ${QT_LIBRARIES}
)

install(TARGETS
  vision_comm
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
