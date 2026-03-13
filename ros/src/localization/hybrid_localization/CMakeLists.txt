cmake_minimum_required(VERSION 3.16)
project(hybrid_localization)

# --- 공통 빌드 옵션 ---
find_package(autodriver_cmake REQUIRED)
include(AutodriverCompilerOptions)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

# --- ESKF 코어 라이브러리 (ROS 의존성 없음, 재사용 가능) ---
ament_auto_add_library(eskf_core SHARED
  src/eskf/eskf_core.cpp
  src/eskf/gnss_update_handler.cpp
  src/eskf/odom_builder.cpp
)
target_include_directories(eskf_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(eskf_core Eigen3::Eigen)

# --- 전처리기 라이브러리 ---
ament_auto_add_library(localization_preprocess SHARED
  src/preprocess/imu_preprocessor.cpp
  src/preprocess/gnss_preprocessor.cpp
  src/preprocess/gnss_heading_arbitrator.cpp
  src/preprocess/imu_calibration_manager.cpp
)
target_include_directories(localization_preprocess PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(localization_preprocess Eigen3::Eigen)

# --- 유틸리티 라이브러리 ---
ament_auto_add_library(localization_util SHARED
  src/util/map_projector.cpp
  src/util/tf_cache.cpp
  src/util/diagnostics_publisher.cpp
)
target_include_directories(localization_util PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(localization_util Eigen3::Eigen)

# --- FGO 라이브러리 (Stage 2: KeyframeBuffer + ImuPreintegration) ---
ament_auto_add_library(fgo_core SHARED
  src/fgo/imu_preintegration.cpp
  src/fgo/keyframe_buffer.cpp
)
target_include_directories(fgo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fgo_core Eigen3::Eigen)

# --- 노드 실행 파일 ---
ament_auto_add_executable(hybrid_localization_node
  src/main.cpp
  src/hybrid_localization_node.cpp
  src/parameters.cpp
  src/node_params.cpp
  src/cb_imu.cpp
  src/cb_gnss.cpp
  src/cb_vehicle.cpp
  src/cb_initialpose.cpp
  src/publish.cpp
)
target_include_directories(hybrid_localization_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(hybrid_localization_node
  eskf_core
  localization_preprocess
  localization_util
  fgo_core
  Eigen3::Eigen
)

# --- 설치 ---
install(TARGETS
  eskf_core
  localization_preprocess
  localization_util
  fgo_core
  hybrid_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_eskf_core test/test_eskf_core.cpp)
  target_link_libraries(test_eskf_core eskf_core Eigen3::Eigen)

  ament_add_gtest(test_imu_preintegration test/test_imu_preintegration.cpp)
  target_link_libraries(test_imu_preintegration fgo_core Eigen3::Eigen)
endif()

ament_auto_package()
