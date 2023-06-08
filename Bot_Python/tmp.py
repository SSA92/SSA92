    # 파라미터 추가
    def __init__(self):    
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        self.stop_count = 0
        ## 역주행 관련 파라미터
        self.reverse_drive = 0
        self.reverse_steer = 0
        
        
        ### control_driving 마지막 부분에 추가
        # 충돌확인
        if sensing_info.lap_progress > 0.5 and -1 < sensing_info.speed < 1 and not self.is_accident:
            self.accident_count += 1

        # 충돌인지
        if self.accident_count > 7:
            # print("사고발생")
            self.is_accident = True

        # 후진
        if self.is_accident:
            # print("후진")
            # 잔디에 거꾸로 박혔을 때
            # (to_middle로 판별)
            # print("미들", sensing_info.to_middle)
            # if sensing_info.to_middle <= -10:
            #     set_steering = -0.7
            #     set_throttle = -1
            # elif -10 < sensing_info.to_middle < 0:
            #     set_steering = 0
            #     set_throttle = -1
            # elif sensing_info.to_middle >= 10:
            #     set_throttle = -1
            #     set_steering = 0.7
            # elif 0 < sensing_info.to_middle < 10:
            #     set_steering = -0
            #     set_throttle = -1
            
            # 복구 안되고 있을때 (== 후진해도 소용없을 때?)
            if self.stop_count > 30:
                # 직진시도
                set_throttle = 0.5
                if sensing_info.to_middle < 0:
                    set_steering = -0.4
                else:
                    set_steering = 0.4
            # 박고 뒤 돌았을때 -> 역주행코드로 대체

            # 일반적인 경우
            else:
                set_steering = 0.05
                set_throttle = -1
                set_brake = 0
            self.recovery_count += 1
            self.stop_count += 1
            print("후진", self.stop_count)

        # 차량 안밀리게 어느정도 후진하면 가속으로 상쇄
        if self.recovery_count > 7:
            set_throttle = 1

        # 다시 진행
        if self.recovery_count > 10:
            # print("다시시작")
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0
            set_throttle = 1
            print("미들", sensing_info.to_middle)
            # 도로 밖에서 다시 시작하면 조향하면서 가속 (스피드맵 : 8 , 싸피맵 : 11)
            if sensing_info.to_middle > 8:
                set_steering = 0.5
            elif sensing_info.to_middle < -8:
                set_steering = -0.5
            else:
                set_steering = 0
            
        if sensing_info.speed > 40:
            self.stop_count = 0
            
            
        if not sensing_info.moving_forward and not self.is_accident and (self.accident_count + self.recovery_count) < 3 and sensing_info.speed > 0:
            # print("역주행", self.reverse_drive)
            self.reverse_drive += 1
            if not self.reverse_steer:
                if sensing_info.to_middle < 0:
                    self.reverse_steer = -1
                else:
                    self.reverse_steer = 1
            
        if sensing_info.moving_forward:
            self.reverse_drive = 0
            self.reverse_steer = 0


        # 역주행
        if self.reverse_drive > 5:
            # print("역주행 수정")
            set_steering = self.reverse_steer
            set_throttle = 0.5
