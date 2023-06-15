from DrivingInterface.drive_controller import DrivingController
import math

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        
        self.is_last_corner = 0

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

        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.25

        ###########################################################################
        ### 도로 정보 좌표변환
        A = 90
        track_forward_angles = [0,] + sensing_info.track_forward_angles
        distance_to_way_points = [sensing_info.to_middle,] + sensing_info.distance_to_way_points
        Angle_list = [0,]
        
        X_list = []
        Y_list = []

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

        Angle_list = Angle_list[1:]
            
        ###########################################################################

        target = min(int(sensing_info.speed / 50) + 1, 3)

        if sensing_info.speed < 120:
            target = 6
        elif sensing_info.speed < 140:
            target = 7
        else:
            target = 8

        set_steering = math.degrees(math.atan((X_list[target] - half_load_width * 0.65) / Y_list[target])) - sensing_info.moving_angle 
        set_steering /= map_value(sensing_info.speed, 0, 200, 200, (sensing_info.speed + 0.001) // 1.6)

        if self.is_debug:
            def round3(n):
                return round(n,3)
            
            print('Angle_list : ', list(map(lambda x : round(x - 90, 3), Angle_list)))
            print('X_list : ', list(map(round3, X_list)))
            print('Y_list : ', list(map(round3, Y_list)))

            print('target : ', target)
            print('target X : ', X_list[target], ', target Y : ', Y_list[target], ', to middle : ', sensing_info.to_middle)

            print('사이각도 : ', Angle_list[target] - 90)
            print('사이 거리 1 : ', math.sqrt(X_list[target] ** 2 + Y_list[target] ** 2))
            print('직선거리 : ', sensing_info.distance_to_way_points[target])
            print('핸들 값 : ', set_steering)


        ###########################################################################

        ## 긴급 및 예외 상황 처리 ########################################################################################
        full_throttle = True

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        X = abs(max(map_value(sensing_info.speed,0,180,35,17.5), 17))
        road_range = int(sensing_info.speed / X)
        max_angle = 0
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            max_angle = max(max_angle, fwd_angle)
            if fwd_angle > 45:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False
                self.is_last_corner = 3

        ## brake, throttle 제어
        if full_throttle == False:
            target_speed = map_value(max_angle, 0, 90, 90, 50)
            set_brake = map_value(sensing_info.speed - target_speed, 0, 160, 0, 1)
        
        if self.is_last_corner > 0:
            print(self.is_last_corner)
            set_steering += 0.080 * self.is_last_corner / 3
            self.is_last_corner -= 1

        set_throttle = 1.0
        set_brake = 0.0

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
    

# 정규화 함수
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