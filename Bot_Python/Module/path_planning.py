from DrivingInterface.drive_controller import DrivingController

import math
from math import atan2, asin, tan, pi, cos, sin, ceil, sqrt
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
        
        self.is_collided = False
        self.collided_counter = 20
        #
        # Editing area ends
        # ==========================================================#
        super().__init__()
    def PID_controller(self, _input, target, target_params, factor ,kP, kI, kD):
        
        dt = 0.1 # 신호가 0.1초마다 업데이트
        if target_params == 'cornering':
            params = self.cornering_params
        elif target_params == 'positioning':
            params = self.position_params
            
        I_ = params['I']
        prev_E_ = params['prev_E']
        
        E_ax = _input + target*(factor + 0.001)
        I_ += dt * E_ax
        
        I_ = max(min(1, I_), -1)
        dE_ax = (E_ax - prev_E_) / dt
        # Clamp I_ax_integral to be between -1.0 and 1.0
        params['prev_E'] = E_ax
        params['I'] = I_

        Cont_ax = kP * E_ax + kI * I_ + kD * dE_ax
        Cont_ax /= (factor + 0.001)
        return Cont_ax
    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        ################################################################################################################
        
        
        ### 함수 설정
        def plot():
            def round3(n):
                return round(n,3)
            # 삼각 함수에 넣을때는 라디안으로
            A = 90
            track_forward_angles = [0,] + sensing_info.track_forward_angles
            distance_to_way_points = [sensing_info.to_middle,] + sensing_info.distance_to_way_points
            Angle_list = [0,]
            
            self.Road = []
            X_list = []
            Y_list = []

            self.Right_line = []
            Right_road_X = []
            Right_road_Y = []

            self.Left_line = []
            Left_road_X = []
            Left_road_Y = []

            for i in range(20):
                C = 180 - (track_forward_angles[i + 1] - track_forward_angles[i]) - A

                temp = distance_to_way_points[i] * math.sin(math.radians(C)) / distance_to_way_points[i + 1]
                
                temp = min(max(temp, -1), 1)
                A = math.degrees(math.asin(temp))

                car_to_point_angle = 180 - C - A

                Angle_list.append(car_to_point_angle + Angle_list[i])

                X = -math.cos(math.radians(Angle_list[i + 1])) * distance_to_way_points[i + 1]
                Y = math.sin(math.radians(Angle_list[i + 1])) * distance_to_way_points[i + 1]

                X_list.append(X)
                Y_list.append(Y)

                self.Road.append((X, Y))

                Right_road_X.append(X + math.cos(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit)
                Right_road_Y.append(Y - math.sin(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit)

                self.Right_line.append((X + math.cos(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit, Y - math.sin(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit))

                Left_road_X.append(X - math.cos(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit)
                Left_road_Y.append(Y + math.sin(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit)

                self.Left_line.append((X - math.cos(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit, Y + math.sin(math.radians(sensing_info.track_forward_angles[i])) * self.half_road_limit))




            if self.is_debug:
                print()
                print(len(Angle_list))
                print('Angle_list : ', list(map(round3, Angle_list)))
                print('X_list : ', list(map(round3, X_list)))
                print('Y_list : ', list(map(round3, Y_list)))
            
            # import matplotlib.pyplot as plt
            # plt.cla()
            # plt.plot(X_list, Y_list, 'b-', marker='o')
            # plt.plot(Left_road_X, Left_road_Y, 'r-', marker='o')
            # plt.plot(Right_road_X, Right_road_Y, 'r-', marker='o')
            # plt.show(block=False)
            # plt.pause(0.1)
        
        # # print("중앙선",self.Road)
        # # print("왼쪽경계",self.Left_line )
        # # print("오른경계",self.Right_line)
        def main_route(e):
            pathX = []
            pathY = []
            pre_w = np.array([0,0])

            for i in range(19): # y 방향이 직선구간(cos), x방향은 좌우(sin), 차량의 위치는 0,0
                xl, yl = self.Left_line[i]
                xr, yr = self.Right_line[i]
                dx, dy = xl - xr, yl - yr
                
                xl1, yl1 = self.Left_line[i+1]
                xr1, yr1 = self.Right_line[i+1]
                dx1, dy1 = xl1 - xr1, yl1 - yr1
                
                dxi = (xr1 - xr) * (sin(sensing_info.track_forward_angles[i+1] * (pi/180)) - sin(sensing_info.track_forward_angles[i] * (pi/180)))
                dyi = (yr1 - yr) * (cos(sensing_info.track_forward_angles[i+1] * (pi/180)) - cos(sensing_info.track_forward_angles[i] * (pi/180)))
                
                
                r = (sensing_info.track_forward_angles[i+1]-sensing_info.track_forward_angles[i])/10 # 곡률
                # print(dxi, xr1-xr, xl1-xl)
                # print(dyi, yr1-yr, yl1-yl)
                
                H11 = dx1**2 + dy1**2 
                H22 = dx**2 + dy**2
                H12 = -1*(dx1*dx + dy1*dy)
                
                B11 = 2*(dxi*dx1 + dyi*dy1)
                B12 = -2*(dxi*dx + dyi*dy)

                Hs = np.array([[H11, H12], [H12, H22]])
                Bs = np.array([B11, B12])
                
                try : # 역행렬이 있는경우
                    w = np.linalg.inv(Hs).dot(Bs)
                    pre_w = w
                except: # 역행렬이 없는경우
                    w = pre_w
                    
                x,y = self.Road[i+1]
                
                # print(w)
                pathX.append(max(min(xr + w[0]*(xl-xr), xr), xl))
                pathY.append(max(min(yr + w[1]*(yl-yr), yr), yl))

                return pathX, pathY

        ## 함수 설정 끝
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

        ##########################################################################
        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.25

        ## 0. 기본값 세팅
        angle_num = int(sensing_info.speed / 45)
        ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0

        ## 0.1 throttle 값
        if sensing_info.speed < 60: set_throttle = 1  
        else : set_throttle =  min( map_value(sensing_info.speed,0,60,0,1), 1)
        
        ## 0.2 break 값
        set_brake = 0
        
        ## 0. 기본값 세팅
        if sensing_info.speed < 10:
            angle_num = 1
        elif sensing_info.speed < 40:
            angle_num = 2
        elif sensing_info.speed < 90:
            angle_num = 3
        elif sensing_info.speed < 140:
            angle_num = 4
        else:
            angle_num = 5

        
        plot()
        pathX, pathY = main_route(e=0)
        ref_angle = math.degrees(math.atan2(pathX[angle_num][0] , pathY[angle_num][1]))
        #ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0
        
        ## 0.3. 차량의 Speed 에 따라서 핸들을 돌리는 값[steer_factor]
        steer_factor = map_value(sensing_info.speed, 0, 200, 200, 60)
        
        ## 0.4 main route로 가기위한 steering angle 설정
        set_steering = ref_angle - sensing_info.moving_angle
        set_steering /= steer_factor + 0.001


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

        ########################################

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


