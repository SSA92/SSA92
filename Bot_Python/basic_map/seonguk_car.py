from DrivingInterface.drive_controller import DrivingController
import math
# import matplotlib.pyplot as plt

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        
        ## 코너링 관련 파라미터    
        self.prev_E_cornering = 0
        self.I_cornering = 0

        self.prev_E_positioning = 0
        self.I_positioning = 0

        self.enter_corner = 0

        self.is_last_corner = 0
        
        self.is_draw = False

        ## 도로 정보
        self.Road = []
        self.Left_line = []
        self.Right_line = []
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False

        # api or keyboard
        self.enable_api_control = True  # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def print(self, text): return self.logger.info(text)

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here

        # 디버그 콘솔 설정 {console: internalConsole, integratedTerminal, externalTerminal}

        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))

            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("=========================================================")

        ###########################################################################

        set_brake = 0.0
        
        ## 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
        if sensing_info.speed < 60: set_throttle = 1  ## 속도가 60Km/h 이하인 경우 0.9 로 설정
        # if sensing_info.speed > 80: set_throttle = 0.8  ## 최대속도를 80km/h로 설정
        else : set_throttle =  min( map_value(sensing_info.speed,0,60,0,1), 1)
        
        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.25

        ## 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
        angle_num = int(sensing_info.speed / 45)
        ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0
        ref_distance = sensing_info.distance_to_way_points[angle_num] if angle_num > 0 else 0
        reli_routeInform = route_info(sensing_info.track_forward_angles)
        
        road_range = int(sensing_info.speed / 45)
        is_corner(sensing_info.track_forward_angles)

        corner_range = int(sensing_info.speed / 20)
        for i in range(0, corner_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if fwd_angle > 20:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                self.enter_corner = 5
                break
        
        # ref_angle -= ready
            
        # else:
        #     # print("코너가 아닙니다.")
        #     self.I_cornering = 0
        #     self.prev_E_cornering = 0

        #     ## 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
        #     ref_mid = (sensing_info.to_middle / (sensing_info.speed)) * -1.2

        ## 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
        steer_factor = sensing_info.speed * 1.5
        if sensing_info.speed > 70: steer_factor = sensing_info.speed * 0.9
        if sensing_info.speed > 100: steer_factor = sensing_info.speed * 0.75
        if sensing_info.speed > 150: steer_factor = sensing_info.speed * 0.66
        if sensing_info.speed > 170: steer_factor = sensing_info.speed * 0.5

        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)

        ## 차선 중앙정렬 값을 추가로 고려함

        ###########################################################################
        ### 도로 정보 좌표변환
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

            temp = distance_to_way_points[i] * math.sin(math.radians(C)) / max(0.001,distance_to_way_points[i + 1])
            
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

        Angle_list = Angle_list[1:]
            
        ###########################################################################


        L = 4.6 # 차 길이 (m)
        target = min(int(sensing_info.speed / 50) + 1, 3)

        if sensing_info.speed < 120:
            target = 6
        elif sensing_info.speed < 140:
            target = 7
        else:
            target = 8


        alpha  = Angle_list[target] - 90 - sensing_info.moving_angle
        delta = math.atan2(2 * L * math.sin(math.radians(alpha)), (sensing_info.distance_to_way_points[target])) 

        target_list_X = []
        target_list_Y = []
        for i in range(19):
            def get_R(target):
                theta1 = Angle_list[target]
                theta2 = Angle_list[target + 1]
                R = abs(10/(2*(theta2-theta1)+0.001))
                
                # print("곡률반경 R : ", R)
                return R
            
            R = min(get_R(i), 50)

            go_to = 1 if Angle_list[i] > 90 else -1
            length = map_value(R, 0, 50, 0.9, 0.001) * half_load_width
            
            target_X = (X_list[i] + go_to * math.cos(math.radians(sensing_info.track_forward_angles[i])) * length)
            target_Y = (Y_list[i] - go_to * math.sin(math.radians(sensing_info.track_forward_angles[i])) * length)

            target_list_X.append(target_X)
            target_list_Y.append(target_Y)

        # delta = self.PID_controller(math.radians(sensing_info.moving_angle), delta, self.cornering_param, factor=1, kP=1, kI=0, kD=0)
        # set_steering = max(1, sensing_info.speed*0.02) * delta 
        # set_steering = 1.2 * delta 

        
        # that_is_corner = False
        # corner_range = int(sensing_info.speed / 20)
        # for i in range(0, corner_range):
        #     fwd_angle = abs(sensing_info.track_forward_angles[i])
        #     if fwd_angle > 180: 
        #         that_is_corner = True

        # if that_is_corner:
        #     target -= 3
        #     alpha = math.degrees(math.atan2(target_list_X[target], target_list_Y[target])) - sensing_info.moving_angle
        #     delta = math.atan2(2 * L * math.sin(math.radians(alpha)), math.sqrt(target_list_X[target]**2 + target_list_Y[target]**2)) 
        #     set_steering = delta * sensing_info.speed * 0.010
        # else:
        set_steering = math.atan((X_list[target] - half_load_width * 0.625) / Y_list[target]) * 180 / math.pi - sensing_info.moving_angle 
        set_steering /= map_value(sensing_info.speed, 0, 200, 160, (sensing_info.speed+0.001)//1.45)


        if self.is_debug:
            def round3(n):
                return round(n,3)
            
            print('Angle_list : ', list(map(lambda x : round(x - 90, 3), Angle_list)))
            print('X_list : ', list(map(round3, X_list)))
            print('Y_list : ', list(map(round3, Y_list)))


            print('target : ', target)
            print('alpha : ', alpha)
            print('alpha_rad : ',math.radians(alpha))
            print('delta : ', math.degrees(delta))
            print('target X : ', X_list[target], ', target Y : ', Y_list[target], ', to middle : ', sensing_info.to_middle)
            print('target X : ', target_list_X[target], ', target Y : ', target_list_Y[target])

            print('사이각도 : ', Angle_list[target] - 90)
            print('사이 거리 1 : ', math.sqrt(X_list[target] ** 2 + Y_list[target] ** 2))
            print('직선거리 : ', sensing_info.distance_to_way_points[target])
            print('핸들 값 : ', set_steering)


        # if self.is_draw:
        #     plt.plot(X_list, Y_list, 'b-', marker='o')
        #     plt.plot(Left_road_X, Left_road_Y, 'r-', marker='o')
        #     plt.plot(Right_road_X, Right_road_Y, 'r-', marker='o')
        #     plt.plot(target_list_X, target_list_Y, 'g-', marker='x')
        #     plt.show(block=False)
        #     plt.pause(0.03)
        #     plt.cla()

        ###########################################################################

        ## 긴급 및 예외 상황 처리 ########################################################################################
        full_throttle = True
        emergency_brake = False

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        X = abs(max(map_value(sensing_info.speed,0,180,35,17.5), 17))
        road_range = int(sensing_info.speed / X)
        max_angle = 0
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            max_angle = max(max_angle, fwd_angle)
            if fwd_angle > 48:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False
                self.is_last_corner = 3

            if fwd_angle > 80:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True

        ## brake, throttle 제어
        
        set_brake = 0.0
        if full_throttle == False:

            target_speed = map_value(max_angle, 0, 90, 90, 50)
            set_brake = map_value(sensing_info.speed - target_speed, 0, 160, 0, 1)
            
        set_brake = 0.0

        if self.is_last_corner > 0:
            print(self.is_last_corner)
            set_steering += 0.88 * self.is_last_corner / 3
            # set_brake = 0.4
            self.is_last_corner -= 1

        ################################################################################################################
        set_steering += (sensing_info.to_middle/((sensing_info.speed+0.001)*20)) * -1
        # Moving straight forward
        car_controls.steering = set_steering
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake

        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}" \
                  .format(car_controls.steering, car_controls.throttle, car_controls.brake))

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
    

def route_info(fw_angles):
    sightsee = fw_angles
    rel_YLength = 0
    rel_XLength = 0
    for fw_angle in sightsee:
        rel_YLength += math.sin(fw_angle * (math.pi/180))
        rel_XLength += math.cos(fw_angle * (math.pi/180))
    # print("차량 기준 y값 진행률", rel_YLength)
    # print("차량 기준 x값 진행률", rel_XLength)
    return (rel_XLength, rel_YLength)

def prepare_corner(road_data):
    _is_corner = False
    x, y = road_data
    # if y > 12 and x < 15 : _is_corner = True
    # elif x >= 15 : _is_corner = False
    if abs(y) > 10  : _is_corner = True
    else : _is_corner = False
    return _is_corner

def is_corner(fw_angles):
    theta1 = fw_angles[1]
    theta2 = fw_angles[2]
    R = abs(10/(2*(theta2-theta1)+0.001))
    
    # print("곡률반경 R : ", R)
    return True if R <= 4.9 else False

# 정규화 함수
def map_value(value, min_value, max_value, min_result, max_result):
    '''maps value (or array of values) from one range to another'''
    if (max_value - min_value) == 0: return max_result
    result = min_result + (value - min_value)/(max_value - min_value)*(max_result - min_result)
    return result

# PI제어기
def PID_controller(_input, target, I_, prev_E_, factor, kP, kI, kD):
    
    dt = 0.1 # 신호가 0.1초마다 업데이트
    
    E_ax = _input + target*(factor + 0.001)
    I_ += dt * E_ax
    dE_ax = (E_ax - prev_E_) / dt
    # Clamp I_ax_integral to be between -1.0 and 1.0
    prev_E_ = E_ax

    Cont_ax = kP * E_ax + kI * I_ + kD * dE_ax
    Cont_ax /= (factor + 0.001)
    return Cont_ax

def move_to_out(car_speed, road_data, car_yaw, car_pos, half_road_width, i_cornering, prev_e_cornering):
    x, y = road_data
    
    # y가 음수(좌측) 일 경우에는 +1, y가 양수(우측) 일 경우에는 -1
    out_is = 1 if y <0 else -1
    P = abs(map_value(x, 0, 20, 2, 1))
    I = abs(map_value(y, 0, 20, 1, 1.3))
    proximate_dist = ((car_speed /3.6)+ 4.001)
    target_pos = out_is * (half_road_width - 3)
    next_pos = PID_controller(car_pos, target_pos * -1 , i_cornering, prev_e_cornering, factor=1, kP=P, kI=I, kD=0.0)
    
    target_angle = _required_angle(proximate_dist+x, car_pos, next_pos)
    next_angle = PID_controller(car_yaw, target_angle , i_cornering, prev_e_cornering, factor=1, kP=1, kI=1, kD=0)
    
    return next_angle

def move_to_in(car_speed, road_data, car_yaw, car_pos, half_road_width, i_cornering, prev_e_cornering):
    x, y = road_data
    # print("테스트 : ", sqrt(x**2+y**2))
    # y가 음수(좌측) 일 경우에는 +1, y가 양수(우측) 일 경우에는 -1
    out_is = 1 if y <0 else -1
    P = abs(map_value(x, 0, 20, 2, 1))
    I = abs(map_value(y, 0, 20, 1, 1.5))
    proximate_dist = ((car_speed /3.6)+ 4.001)
    # print(f"{proximate_dist}m/s")
    target_pos = out_is * (half_road_width - 1.5)
    next_pos = PID_controller(car_pos, target_pos , i_cornering, prev_e_cornering, factor=1, kP=P, kI=I, kD=0.01)
    
    target_angle = _required_angle(125, car_pos, next_pos)
    next_angle = PID_controller(car_yaw, target_angle , i_cornering, prev_e_cornering, factor=1, kP=1, kI=1, kD=0.01)
    next_angle +=( math.atan2(y,x)* (180/math.pi) - 90) / 40
    return next_angle

def _required_angle(proximate_dist, car_pos, target_pos):
    target_angle = (math.atan2(proximate_dist, (car_pos - target_pos)) * (180/math.pi) - 90 )
    if abs(car_pos - target_pos) <= 0.2:
        return  target_angle + 0.5 if target_angle >= 0  else target_angle - 0.5
    return target_angle

if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)