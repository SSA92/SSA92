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

**공통**: 
- 디버깅 및 테스트 
- 가중치 수정


### 1. 상세 업무 분담 내역
---
<br>

### 1.1 **Path planning** 
---
> 목표 : 경로 최적화를 통하여, 감속을 최소화 하여 경로 주행

<br>

### 1.2 **장애물 회피 로직** 
---
> 목표 : 제공받는 장애물 자료를 바탕으로 안정적인 경로 탐색 및 제어시 발생하는 감속 최소화
 
<br>

### 1.3 **Ego Vehicle 복구** 
---
> 목표 : 충돌로 인한 차량 정지 시, 차량의 빠른 경로 복구

<br>

### 참고자료
- [로보틱스 주행계획 참고자료](https://github.com/AtsushiSakai/PythonRobotics)
- [The Vector Field Histogram - Fast Obstacle Avoidance for Mobile Robots 정리 자료](https://soohwan-justin.tistory.com/26)
- [Vector Field Histogram](https://www.mathworks.com/help/nav/ug/vector-field-histograms.html)
- 무인 자율 주행을 위한 최단 시간 경로계획 알고리즘 설계. 김동욱 저

