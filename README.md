# SSA92
<br>

### 프로젝트 기간 🗓️
- 2023.05.29 ~ 2023.06.29

<br>

### 프로젝트 개요 🌟
- Microsoft AirSim을 활용한 혁신적인 차량 운행 시뮬레이션 대회 참여
- 직접 개발한 알고리즘을 통해 가상의 차량을 주행시키며, 완주 시간을 기준으로 성적을 겨룸
- 차량의 위치, 속도, 방향 등을 실시간으로 분석하여 최적의 주행 전략을 세우는 것이 핵심

### 개발환경 🛠️
>
> Python 3.9.
> >
> Unity

<br>

## 0. 팀원 🧑‍🤝‍🧑

| 팀구성 | 역할 | 상세 업무 분담 내역|
|:---|:---|:---|
|김수찬|팀장| **장애물 회피** 및 Path planning |
|최영태|팀원| **Ego 복구 로직** 및 장애물 회피 |
|박성욱|팀원| **Path planning** 및 모듈 Merge|

**자율주행 구현 목표**: 
[기본 요구사항]
도로의 형태에 따라 알맞은 주행을 통해, 대상 차량의 자율주행에 안정성을 확보
자율 주행 제어는 총 세 가지 값으로 결정

1. 방향 제어 (steering)
   - 값이 0보다 크면 오른쪽 방향으로, 값이 0보다 작으면 왼쪽 방향으로 제어
   - 값의 범위는 -1 에서 +1 사이의 값
   - 실제 차량 모델에서 타이어의 회전각은 약 -45º 에서 +45º 의 값이다.
  
2. 속도 제어 (Accelerator)
   - 값이 0보다 크면 전진으로, 값이 0보다 작으면 후진으로 제어
   - 0 보다 클 때, 기어는 속도에 따라 자동으로 변속이 이루어 진다.
   - 값의 범위는 -1 에서 +1 사이의 값이다.

3. 정지 제어 (brake)
   - 값이 0보다 크면 차량이 감속 또는 정지하도록 제어가 이루어 진다.
   - 값의 범위는 0 에서 +1 사이의 값이다

해당 3가지 제어를 분담에 맞게 값을 조정할 수 있도록 작업
1. **Path Planning**
2. **Local Path Planning**
3. **Ego Vehicle Recovery System**

### 1. 상세 업무 분담 내역
---
<br>

### 1.1 **Path planning** 
---
> 목표 : 경로 최적화를 통하여, 감속을 최소화 하여 경로 주행

<br>

### 1.2 **Local Path Planning** 
---
> 목표 : 제공받는 장애물 자료를 바탕으로 안정적인 경로 탐색 및 제어시 발생하는 감속 최소화
 
<br>

### 1.3 **Ego Vehicle Recovery System** 
---
> 목표 : 충돌로 인한 차량 정지 시, 차량의 빠른 경로 복구

<br>

### 참고자료
- [로보틱스 주행계획 참고자료](https://github.com/AtsushiSakai/PythonRobotics)
- [The Vector Field Histogram - Fast Obstacle Avoidance for Mobile Robots 정리 자료](https://soohwan-justin.tistory.com/26)
- [Vector Field Histogram](https://www.mathworks.com/help/nav/ug/vector-field-histograms.html)
- 무인 자율 주행을 위한 최단 시간 경로계획 알고리즘 설계. 김동욱 저

