# 🚁 Sim2Real-Drone-RL: 강화학습 기반 드론 자율비행 시스템  

> **상세 아키텍처 및 트러블슈팅(문제 해결) 과정은 [https://www.notion.so/31597446a87a80748027cb33b7e4dddb?source=copy_link]에서 확인하실 수 있습니다.**  

## 1. Project Overview  
기존의 정적 데이터 기반 딥러닝을 넘어, 동적 환경에서의 자율적인 강화학습(RL) 방법론을 연구하고 이를 실제 물리적 모듈에 적용하는 Sim2Real 프로젝트입니다.   
하드웨어의 연산 한계를 극복하기 위해 **연산 환경(Raspberry Pi)과 제어 환경(ESP32)을 분리한 클라이언트-서버 구조의 분산 처리 아키텍처**를 설계했습니다.  

## 2. Tech Stack  
* **Embedded / H/W**: `ESP32`, `C++ (Arduino IDE)`, `FreeRTOS`  
* **AI & Simulation**: `Unity 3D`, `ML-Agents`, `Python`, `ONNX`  
* **Communication**: `BLE (Bluetooth Low Energy)`, `I2C`  

## 3. Directory Structure  
```text  
📦 Sim2Real-Drone-RL  
 ┣ 📂 Drone_Firmware      # ESP32 FreeRTOS 기반 펌웨어 제어 코드 (C++)  
 ┃ ┗ 📜 drone_control.cpp # 200Hz PID 제어 및 비행 태스크 로직  
 ┣ 📂 Simulation_Env      # Unity 3D 및 ML-Agents 환경 세팅 (C#, YAML)  
 ┃ ┣ 📜 DroneAgent.cs     # 에이전트 행동 및 보상(Reward) 설계 로직  
 ┃ ┣ 📜 config.yaml       # PPO 알고리즘 하이퍼파라미터 튜닝 설정값  
 ┃ ┗ 📜 model.onnx        # 최종 학습 완료된 추론 모델  
 ┗ 📜 README.md  
