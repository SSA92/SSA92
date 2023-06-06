from DrivingInterface.drive_controller import DrivingController
import math

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = True

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

        ## 도로의 실제 폭의 1/2 로 계산됨 ( 도로폭 이용 필요 )
        half_load_width = self.half_road_limit - 1.25

        ## 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기 ( 전방 전체를 참고하면 좋을듯 )
        angle_num = int(sensing_info.speed / 40)
        ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0
        
        ## 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
        ref_mid = (sensing_info.to_middle / 80) * -1 # 오른쪽에있으면 핸들 왼쪽으로 === *-1

        ## 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함

        if sensing_info.speed < 60: set_throttle = 1.0  ## 속도가 60Km/h 이하인 경우 1.0 로 설정
        # 60 이상일때는 정규화로 하는데 항상1이네
        else : set_throttle =  min( self.map_value(sensing_info.speed,0,60,0,1), 1)

        ## 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
        steer_factor = sensing_info.speed * 1.5
        if sensing_info.speed > 70: steer_factor = sensing_info.speed * 0.9
        if sensing_info.speed > 100: steer_factor = sensing_info.speed * 0.66

        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        set_steering = (ref_angle - sensing_info.moving_angle) / 90

        ## 차선 중앙정렬 값을 추가로 고려함
        # if ref_angle == 0:
        # gen_ref_angle = self.map_value(ref_angle,-50,50,0,1)
        # middle_add = gen_ref_angle * ref_mid
        # set_steering += middle_add
        set_steering += ref_mid

        ###########################################################################
        # try:
        L = 4.6 # 차 길이 (m)
        # 삼각 함수에 넣을때는 라디안으로

        C = 90 - sensing_info.track_forward_angles[0]
        temp = sensing_info.to_middle * math.sin(math.radians(C)) / sensing_info.distance_to_way_points[0]
        
        temp = min(max(temp, -1), 1)

        A = math.degrees(math.asin(temp))
        car_to_point_angle = 180 - C - A
        Angle_list = [car_to_point_angle]

        X = -math.cos(math.radians(Angle_list[0])) * sensing_info.distance_to_way_points[0]
        Y = math.sin(math.radians(Angle_list[0])) * sensing_info.distance_to_way_points[0]

        X_list = [X]
        Y_list = [Y]

        for i in range(19):
            C = 180 - (sensing_info.track_forward_angles[i + 1] - sensing_info.track_forward_angles[i]) - A
            temp = sensing_info.distance_to_way_points[i] * math.sin(math.radians(C)) / sensing_info.distance_to_way_points[i + 1]
            
            temp = min(max(temp, -1), 1)

            A = math.degrees(math.asin(temp))
            car_to_point_angle = 180 - C - A

            Angle_list.append(car_to_point_angle + Angle_list[i])

            X = -math.cos(math.radians(Angle_list[i + 1])) * sensing_info.distance_to_way_points[i + 1]
            Y = math.sin(math.radians(Angle_list[i + 1])) * sensing_info.distance_to_way_points[i + 1]

            X_list.append(X)
            Y_list.append(Y)

            # print(car_to_point_angle)
            # print('X, Y : ', X, Y)

        def round3(n):
            return round(n,3)

        target = max(int(sensing_info.speed / 20), 5)
        
        print('Angle_list : ', list(map(round3, Angle_list)))
        print('X_list : ', list(map(round3, X_list)))
        print('Y_list : ', list(map(round3, Y_list)))
        print('target X : ', X_list[target], ', target Y : ', Y_list[target])
        print('사이각도 : ', sensing_info.track_forward_angles[target])
        print('직선거리 : ', sensing_info.distance_to_way_points[target])

        plt.plot(X_list, Y_list, '-', marker='o')
        plt.show(block=False)

        # 왜 안되지
        # https://iridescentboy.tistory.com/6
        delta = math.atan(2 * X_list[target] * L / (sensing_info.distance_to_way_points[target] ** 2))
        print('delta : ', delta, '(rad)')
        print('delta : ', math.degrees(delta) - sensing_info.moving_angle, '(deg)')

        # a = math.asin(sensing_info.distance_to_way_points[0] / (2 * max(X, Y))) * 2
        # b = a * sensing_info.speed * 0.1 / max(X, Y)
        # b = b if math.atan(Y / X) * 180 / math.pi - sensing_info.moving_angle >= 0 else - b

        print(set_steering)
        # if sensing_info.track_forward_angles[target] > 30:
        #     set_steering = math.radians(math.degrees(delta) - sensing_info.moving_angle)
        set_steering = (math.degrees(delta) - sensing_info.moving_angle) / max(85, sensing_info.speed * 0.95)
        set_steering = math.radians(math.degrees(delta) - sensing_info.moving_angle)
        set_steering = 0
        print(set_steering)

        # except:
        #     pass

        ###########################################################################

        ## 긴급 및 예외 상황 처리 ########################################################################################
        full_throttle = True
        emergency_brake = False

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        road_range = int(sensing_info.speed / 20)
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if fwd_angle > 45:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False
            if fwd_angle > 80:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True
                break

        ## brake, throttle 제어
        
        set_brake = 0.0
        if full_throttle == False:
            # print(sensing_info.moving_angle)

            # set_brake = min(0.35 + self.map_value(abs(sensing_info.moving_angle),0,50,0,1),1)
            set_brake = min(self.map_value(abs((ref_angle - sensing_info.moving_angle)),0,90,0,1),1)

            # print(set_brake)
            # if sensing_info.speed > 100:
            #     set_brake = 0.3
            # if sensing_info.speed > 120:
            #     set_throttle = 0.9
            #     set_brake = 0.4
            # if sensing_info.speed > 130:
            #     set_throttle = 0.8
            #     set_brake = 0.5

        ################################################################################################################

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

    # PI제어기
    def PI_controller(self):
        I_ax = 0.1 # 신호가 0.1초마다 업데이트
        E_ax = udp_input.DriveCont.Ax - car.ConBdy1.a_1[0]
        I_ax_integral += I_ax * E_ax

        # Clamp I_ax_integral to be between -1.0 and 1.0
        I_ax_integral = min(max(I_ax_integral, -1.0), 1.0)

        Cont_ax = P_ax * E_ax + I_ax_integral
        return Cont_ax


    # 정규화 함수
    def map_value(self, value, min_value, max_value, min_result, max_result):
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