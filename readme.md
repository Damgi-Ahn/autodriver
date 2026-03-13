# Autodriver

Autodriver는 ROS 2 기반의 자율주행/로보틱스 파이프라인을 실험적으로 구현하는 프로젝트입니다.

> 본 프로젝트는 **토이 프로젝트**이며, **Claude를 활용해 개발**하고 있고, 현재 **검증(Validation) 진행 중**인 상태입니다.

## 프로젝트 개요

이 저장소는 Jetson 계열 환경을 고려한 고성능 인식/시스템 파이프라인을 ROS 2 패키지 형태로 구성합니다.
핵심 목적은 다음과 같습니다.

- 다중 카메라 입력 수집 및 파이프라인 관리
- TensorRT 기반 추론 스케줄링/실행
- GPU/파이프라인 상태 모니터링
- 위치 추정(Localization) 모듈 실험

## 코드 구조 분석

현재 코드는 기능별 ROS 2 패키지로 분리되어 있습니다.

- `ros/src/sensor_component/camera_manager`
  - 카메라 설정 로딩, 파이프라인 구성, 노드 실행
- `ros/src/perception/tensorrt_inference_manager`
  - 모델 실행, 스케줄링, NV 버퍼 풀 관리
- `ros/src/system/*`
  - `gpu_watchdog`, `metrics_manager`, `pipeline_profiler` 등 시스템 상태/성능 관리
- `ros/src/localization/hybrid_localization`
  - IMU/GNSS 전처리 및 ESKF 기반 하이브리드 위치 추정
- `ros/src/autodriver_bringup`
  - 전체/부분 시스템 launch 구성
- `docs/`
  - 아키텍처, 노드별 동작, 라이브러리 문서

## 빠른 시작

```bash
cd ros
colcon build --symlink-install
source install/setup.bash
```

시스템 실행 예시:

```bash
ros2 launch autodriver_bringup autodriver.launch.xml
```

## 문서

상세 설계 및 패키지 설명은 `docs/README.md`부터 확인할 수 있습니다.
