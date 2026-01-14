#  STM32 기반 실시간 화재 감시 및 진압 RC카 (Fire Detection RC Car)

### "골든타임 확보를 위한 초기 대응 시스템"

사람의 접근이 어려운 사각지대에서 화재 발생 시, 신속하게 이를 감지하고 초기 진압을 수행할 수 있는 **STM32 기반 임베디드 시스템**입니다.
단순한 RC카 구동을 넘어, **Bare-Metal 기반의 펌웨어 설계**를 통해 센서 데이터 처리의 실시간성을 확보하고, 하드웨어 노이즈를 제어하여 시스템의 안정성을 높였습니다.

---

##  Tech Stack

| Category | Technology |
| :--- | :--- |
| **MCU** | ![STM32](https://img.shields.io/badge/STM32F103RB-03234B?style=flat-square&logo=stmicroelectronics&logoColor=white) (ARM Cortex-M3) |
| **Language** | ![C](https://img.shields.io/badge/C-A8B9CC?style=flat-square&logo=c&logoColor=white) (HAL Library) |
| **Development** | ![CubeIDE](https://img.shields.io/badge/STM32CubeIDE-03234B?style=flat-square&logo=stmicroelectronics&logoColor=white) |
| **Sensor/Actuator** | KY-026 (Fire), L298N (Motor Driver), SG90 (Servo), HC-06 (Bluetooth) |
| **Server** | ![Node.js](https://img.shields.io/badge/Node.js-339933?style=flat-square&logo=node.js&logoColor=white) (Web Control Interface) |

---

## 🔌 System Architecture

<img width="1102" height="549" alt="image" src="https://github.com/user-attachments/assets/a63c4691-e497-496b-8582-f861b2ad31fc" />


* **Main Controller:** STM32F103RB (ADC/PWM/UART 제어)
* **Actuator:** DC 모터(주행), 서보 모터(소화 장치 조준/발사)
* **Sensor:** 불꽃 감지 센서 (Analog/Digital 인터페이스)
* **Comm:** Bluetooth UART 통신 (Mobile/Web 연동)

---

##  Key Functions

### 1. 실시간 화재 감지 및 인터럽트 처리 (Fire Detection)
* **ADC & Polling:** KY-026 센서의 아날로그 값을 ADC로 변환하여 불꽃의 세기를 정밀 측정합니다.
* **Interrupt Handling:** 임계값 이상의 신호가 감지되면 즉시 `EXTI` 인터럽트를 발생시켜 주행 모터 동작을 중단하고 경보(Buzzer) 및 소화 모드로 전환합니다.
* **Safety Logic:** 센서 오작동 방지를 위해 연속 3회 이상 감지 시에만 확정 신호로 처리하는 디바운싱(Debouncing) 로직을 구현했습니다.

### 2. 타이머 기반 정밀 모터 제어 (Motor Control)
* **DC Motor:** `TIM3`의 PWM 채널을 활용하여 Duty Cycle 조절을 통해 부드러운 가속/감속 주행을 구현했습니다.
* **Servo Motor (소화 기능):** `TIM2`를 사용하여 모의 소화탄 발사 장치의 각도를 정밀 제어합니다. (장전-조준-발사 시퀀스 자동화)

### 3. UART 통신 및 원격 관제
* **Non-Blocking 통신:** 링 버퍼(Ring Buffer)와 UART 인터럽트를 사용하여, 주행 제어 중에도 끊김 없이 관제 서버(Web)와 데이터를 주고받도록 설계했습니다.
* **Protocol:** 경량화된 패킷 구조(Header/Data/Checksum)를 정의하여 통신 신뢰성을 확보했습니다.

---

##  Troubleshooting (핵심 문제 해결)

### 1. 외부 광원 간섭 및 감지 각도에 따른 센서 오작동
* **문제 상황 (Issue):** 화재 감지 센서(KY-026)가 햇빛이나 형광등 같은 외부 광원에 반응하거나, 특정 각도에서만 불꽃을 인식하여 오작동(False Positive) 및 미감지(False Negative) 현상이 발생.
* **접근 방법 (Approach):** 실제 화재를 매번 발생시켜 테스트하기에는 안전상 위험이 따르고, 재현성이 떨어짐을 인지.
* **해결 (Solution):**
    * **가상 테스트 환경 구축:** 아두이노와 IR 송신 모듈을 활용하여, 실제 불꽃과 유사한 파장대를 방출하는 **'가상 화재 시뮬레이터'**를 제작.
    * **검증:** 다양한 거리와 각도에서 IR 신호를 쏘아주며 센서 감도(Sensitivity)를 최적화하고, 외부 광원이 있는 환경에서도 정확히 화재 신호만 걸러내도록 임계값을 튜닝함.

### 2. 유선 카메라로 인한 활동 반경 제한
* **문제 상황 (Issue):** 초기 프로토타입에서 USB 유선 웹캠을 사용했으나, 케이블 길이로 인해 RC카가 이동할 수 있는 물리적 반경이 제한됨. 자율 순찰 로봇의 목적에 부합하지 않음.
* **해결 (Solution):**
    * **무선 스트리밍 전환:** 고가의 무선 카메라 모듈 대신, 안 쓰는 스마트폰과 **'Iriun Webcam'** 어플리케이션을 활용하여 비용 효율적인 무선 영상 전송 시스템 구축.
    * **결과:** 케이블의 제약 없이 Wi-Fi가 닿는 모든 구역으로 순찰 범위를 확장하였으며, 실시간 모니터링 지연(Latency) 문제도 테스트를 통해 안정적인 수준임을 확인함.
  
--- 
##  Results & UI 
### 1. 웹 UI – 순찰 대기 

<img width="400" height="280" alt="image" src="https://github.com/user-attachments/assets/1045a77e-6f25-4bd5-b0f6-272bcbcdfca6" />


### 2. 웹 UI – 자율 순찰 

<img width="400" height="280" alt="image" src="https://github.com/user-attachments/assets/8e182217-daf5-4629-a5b2-424da10e538c" />


### 3. 웹 UI – 화재 감지 후 원격 제어 

<img width="400" height="280" alt="image" src="https://github.com/user-attachments/assets/251f6f02-2458-4a26-bff0-bfd22d50fa24" />



---

##  동작 영상 
### 1. 화재 감시

<img src="images/video_fire_detection.gif" width="400" height="280" />

### 2. 소화 동작

<img src="images/video_extinguish.gif" width="400" height="280" />  

### 3. 순찰 복귀 

 <img src="images/video_patrol_resume.gif" width="400" height="280" /> 

 ---

##  Future Plan
- **RTOS 적용:** FreeRTOS 기반으로 태스크 분리 → 안정성과 실시간성 강화
- - **보드 업그레이드:** STM32F4 등 상위 MCU로 교체 → 성능 및 주변장치 확장
- - **센서 개선:** 불꽃 센서 외 온도/연기 센서 병합 → 오탐지 감소
