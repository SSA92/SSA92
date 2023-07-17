'''
기본 모듈 중 Collided와 관련된 값만 남겨둔 .py 모듈입니다. 실제 구동시 문제가 있을 수 있습니다.
'''


from DrivingInterface.drive_controller import DrivingController

import math
from math import atan2, tan, pi, cos, sin, ceil, sqrt
import numpy as np
# import matplotlib.pyplot as plt


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False

        # api or keyboard
        self.enable_api_control = True  # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        ## 기본 제공 파라미터
        self.track_type = 99
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        
        ## 도로 정보
        self.Road = []
        self.Left_line = []
        self.Right_line = []
        self.Cx = 0
        self.Cy = 0
        
        ## 코너링 관련 파라미터
        self.cornering_params = {
            'prev_E' : 0,
            'I' : 0,
        }
        
        self.position_params = {
            'prev_E' : 0,
            'I' : 0,
        }
        
        self.cornering_start = False
        self.cornering_break = 0.2
        self.cornering_throttle = 0.8       
        ## 역주행 관련 파라미터
        self.reverse_drive = 0
        self.reverse_steer = 0
        self.stop_count = 0
        self.back_dis = 15
        self.steer_list = []
        #
        # Editing area ends
        # ==========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        ################################################################################################################

        ################################################################################################################
        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            # print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            # print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            # print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            # print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################
        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.25

        ## 0. 기본값 세팅
        angle_num = min(int(sensing_info.speed / 45), 3)
        ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0
        
        ## 0.1 throttle 값
        if sensing_info.speed < 60: set_throttle = 1  
        else : set_throttle =  min( map_value(sensing_info.speed,0,60,0,1), 1)
        
        ## 0.2 break 값
        set_brake = 0
        
        ## 0.3. 차량의 Speed 에 따라서 핸들을 돌리는 값[steer_factor]
        value = map_value(sensing_info.speed, 0,200,2,0.75)
        steer_factor = sensing_info.speed * value
        
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)
        

        ## 긴급 및 예외 상황 처리 ########################################################################################

        ########################################
        # 커브각이 클때 쓰로틀 브레이크 설정
        set_throttle = 1.0
        set_brake = 0.0

        full_throttle = True
        road_range = int(sensing_info.speed / 15)
        max_angle = 0
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            max_angle = max(max_angle, fwd_angle)
            if fwd_angle > 50:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False
                self.is_last_corner = 3

        ## brake, throttle 제어
        if full_throttle == False:
            C = 360
            T = 80
            target_speed = map_value(max_angle, 0, 90, C, T)
            if sensing_info.speed - target_speed >= T:
                set_throttle = map_value(sensing_info.speed - target_speed, T, C, 1, 0)
                # print(f"curve brake on [set_throttle :{set_throttle}]")
            set_brake = map_value(sensing_info.speed - target_speed, 0, C, 0, 1)
            # print(f"curve brake on [set_brake : {set_brake}]")

        ######################################################
        # 충돌확인
        if sensing_info.lap_progress > 0.5 and -1 < sensing_info.speed < 1 and not self.is_accident:
            self.accident_count += 1
            # print("collided")
            back_dis = sensing_info.to_middle
            # 충돌 지점에 따라 후진 카운트 조절 (스피드맵 : 7 , 싸피맵 : 10)
            if abs(back_dis) > 7:
                self.back_dis = abs(back_dis) * 2 # (스피드맵 : 1.37 , 싸피맵 : 1.85)

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

        # 차량 안밀리게 어느정도 후진하면 가속으로 상쇄
        if self.recovery_count > 12:
            # print("상쇄")
            set_throttle = 1

        # 다시 진행
        if self.recovery_count >= self.back_dis:
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
            steer = angle * 0.05
            self.steer_list.append(steer)
            # 도로 밖일 때
            if sensing_info.to_middle >= 8 or sensing_info.to_middle <= -8:
                set_steering = -steer
                # print("도로밖")
            elif -8 < sensing_info.to_middle < 8:
                set_steering = steer
                # print("도로 안")

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
            
        if sensing_info.moving_forward and abs(sensing_info.to_middle) > 7.5:
            # print("긴급조향", sensing_info.to_middle, sensing_info.moving_angle)
            if sensing_info.moving_angle > 50:
                set_steering = -0.3
            elif sensing_info.moving_angle < -50:
                set_steering = 0.3
        
            
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

        # if self.is_debug:
        #     print("[MyCar] steering:{}, throttle:{}, brake:{}" \
        #           .format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls
    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name

def map_value(value, min_value, max_value, min_result, max_result):
    '''maps value (or array of values) from one range to another'''
    if (max_value - min_value) == 0: return max_result
    result = min_result + (value - min_value)/(max_value - min_value)*(max_result - min_result)
    return result

if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)


