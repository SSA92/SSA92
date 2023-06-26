## 기본 제공 파라미터
        self.track_type = 99
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

        ## 코너링 관련 파라미터
        self.prev_E_cornering = 0
        self.I_cornering = 0

        ## 역주행 관련 파라미터
        self.stop_count = 0
        self.back_dis = 15
        self.steer_list = []
        self.reverse_drive = 0
        self.reverse_steer = 0
        
        
        # 충돌확인
        if sensing_info.lap_progress > 0.5 and -1 < sensing_info.speed < 1 and not self.is_accident:
            self.accident_count += 1
            # print("collided")
            back_dis = sensing_info.to_middle
            # 충돌 지점에 따라 후진 카운트 조절 (스피드맵 : 7 , 싸피맵 : 10)
            if abs(back_dis) > 10:
                self.back_dis = abs(back_dis) * 1.37 # (스피드맵 : 1.37 , 싸피맵 : 1.85)

        # 충돌인지
        if self.accident_count > 7:
            # print("사고발생", self.accident_count)
            self.is_accident = True

        # 후진
        if self.is_accident:
            # 복구 안되고 있을때 (== 후진해도 소용없을 때?)
            # if self.stop_count > 30:
            #     # 직진시도
            #     set_throttle = 0.5
            #     if sensing_info.to_middle < 0:
            #         set_steering = -0.4
            #     else:
            #         set_steering = 0.4
            # 박고 뒤 돌았을때 -> 역주행코드로 대체

            # 일반적인 경우
            # else:
            set_steering = 0.05
            set_throttle = -1
            set_brake = 0
            self.recovery_count += 1
            self.stop_count += 1
            # print("후진", self.stop_count)

        # 차량 안밀리게 어느정도 후진하면 가속으로 상쇄
        if self.recovery_count > 10:
            # print("상쇄")
            set_throttle = 1

        # 다시 진행
        # 도로 밖에서 후진해올때 도로 중앙근처로 오면 바로 다시시작
        if self.recovery_count >= self.back_dis or (self.back_dis > 15 and abs(sensing_info.to_middle) < 3):
            # print("다시시작", self.recovery_count)
            # print("후진 길이", self.back_dis)
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0
            set_throttle = 1
            # print("미들", sensing_info.to_middle)
            # print("방향", sensing_info.moving_forward)
            # 도로 밖에서 다시 시작하면 도로쪽으로 조향하면서 가속 (스피드맵 : 8 , 싸피맵 : 11)
            angle = sensing_info.moving_angle
            steer = angle * 0.012
            # print(steer, angle)
            self.steer_list.append(steer)
            # 도로 밖일 때
            if sensing_info.to_middle >= 11 or sensing_info.to_middle <= -11:
                set_steering = -steer
                # print("도로밖")
            elif -11 < sensing_info.to_middle < 11:
                set_steering = steer
                # print("도로 안")
            # 도로 오른쪽일 떄
            # if sensing_info.to_middle > 11:
            #     if angle < 0:
            #         set_steering = steer
            #     else:
            #         set_steering = -steer
            # # 도로 왼쪽일 때
            # elif sensing_info.to_middle < -11:
            #     if angle < 0:
            #         set_steering = -steer
            #     else:
            #         set_steering = steer
            else:
                set_steering = 0
            # 복구 안되고 있으면 방향 전환하면서 조향각 증가
            if self.stop_count > 40:
                steer = self.steer_list[-2] * -1.5
                self.steer_list.append(steer)
                set_steering = steer
                # print("조향반대", set_steering)
            
        if sensing_info.speed > 40:
            self.stop_count = 0
            self.back_dis = 15
            self.steer_list.clear
            self.accident_count = 0
            self.recovery_count = 0
            
            
        if not sensing_info.moving_forward and not self.is_accident and (self.accident_count + self.recovery_count) < 7 and sensing_info.speed > 3:
            self.reverse_drive += 1
            # print("역주행", self.reverse_drive)
            if not self.reverse_steer:
                if sensing_info.to_middle < 0:
                    self.reverse_steer = -1
                else:
                    self.reverse_steer = 1
            
        if sensing_info.moving_forward:
            self.reverse_drive = 0
            self.reverse_steer = 0


        # 역주행
        if self.reverse_drive > 5 and not self.is_accident:
            # print("역주행 수정")
            set_steering = self.reverse_steer
            set_throttle = 0.7


        # Moving straight forward
        car_controls.steering = set_steering
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake
