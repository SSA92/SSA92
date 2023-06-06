from DrivingInterface.drive_controller import DrivingController

from math import atan2, pi, sqrt, ceil


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

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

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

        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            # print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            # print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
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
        # print("도로의 폭",self.half_road_limit)
        ## 1. 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
        angle_num = int(sensing_info.speed / 40)
        ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0

        ## 2. 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
        ref_mid = (sensing_info.to_middle / (sensing_info.speed * 0.66 + 0.001)) * -1.1

        ## 3. 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
        if sensing_info.speed < 60:
            set_throttle = 1  ## 속도가 60Km/h 이하인 경우 0.9 로 설정
        # if sensing_info.speed > 80: set_throttle = 0.8  ## 최대속도를 80km/h로 설정
        else:
            set_throttle = min(map_value(sensing_info.speed, 0, 60, 0, 1), 1)

        ## 4. 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
        steer_factor = sensing_info.speed * 1.5
        if sensing_info.speed > 70: steer_factor = sensing_info.speed * 0.9
        if sensing_info.speed > 100: steer_factor = sensing_info.speed * 0.66
        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)

        ## 5. 차량이 정렬되는 중앙정렬 값을 추가적으로 고려함
        gen_ref_angle = map_value(abs(ref_angle), 0, 50, 0, 1) + 0.55
        # gen_ref_angle = map_value(abs(ref_angle),0,50,0,1)

        ### 5.1 장애물 파악
        objects = analyze_obstacles(sensing_info.speed, sensing_info.to_middle, sensing_info.track_forward_obstacles)
        # print(objects)
        ### 5.2 전방 파악
        mapped_data = VFH_grid_map(sensing_info.speed, half_load_width, objects)
        # print("전방 정보", mapped_data)
        ### 5.3 전방 데이터를 바탕으로 경로 설정
        path_recommend = path_planning(sensing_info.speed, sensing_info.to_middle, mapped_data, half_load_width)
        # print("회피 지점",path_recommend + 1)
        ### 5.4 해당 경로를 바탕으로 회피각도 설정
        avoidance_angle = generate_path(sensing_info.speed, sensing_info.to_middle, half_load_width, path_recommend,
                                        objects)
        # print("회피각도", avoidance_angle)

        ## 5.5 경로 설정
        if avoidance_angle:
            selcted_path = 2 * (map_value(abs(avoidance_angle), 0, 50, 0, 1) + 1) * (avoidance_angle) / (
                        steer_factor + 0.001)

            set_steering += selcted_path
        # if ref_angle == 0:
        middle_add = gen_ref_angle * ref_mid
        set_steering += middle_add

        ## 긴급 및 예외 상황 처리 ########################################################################################
        full_throttle = True
        emergency_brake = False

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        road_range = int(sensing_info.speed / 20)
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if fwd_angle > 30:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False
            if fwd_angle > 80:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True
                break

        ## brake, throttle 제어

        set_brake = 0.0
        # if full_throttle == False:
        # print(sensing_info.moving_angle)
        # set_brake = min(map_value(abs(sensing_info.moving_angle),0,50,0,1),1)
        # set_steering *= 1.2
        # print(set_brake)
        if sensing_info.speed > 90:
            if len(objects) > 0:
                set_throttle = 0.83
                set_brake = 0.35
            else:
                set_throttle = 0.9
                set_brake = 0.25

        # 충돌확인
        if sensing_info.lap_progress > 0.5 and -1 < sensing_info.speed < 2 and not self.is_accident:
            self.accident_count += 1

        # 충돌인지
        if self.accident_count > 6:
            # print("사고발생")
            self.is_accident = True

        # 후진
        if self.is_accident:
            # print("후진")
            if sensing_info.to_middle < -5:
                set_steering = -0.3
                set_throttle = 1
            elif -5 < sensing_info.to_middle < 0:
                set_steering = 0.02
                set_throttle = -1
            elif sensing_info.to_middle > 5:
                set_throttle = 1
                set_steering = 0.3
            elif 0 < sensing_info.to_middle < 5:
                set_steering = -0.02
                set_throttle = -1
            set_brake = 0
            self.recovery_count += 1

        # 다시 진행
        if self.recovery_count > 13:
            # print("다시시작")
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0
            set_steering = 0
            set_brake = 0
            set_throttle = 0

        ################################################################################################################

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


## 장애물은 모든 맵 기준 2m 이며 to_middle 기준 좌우 1m입니다.
## 1. 장애물을 분석
def analyze_obstacles(car_speed, car_pos, fw_obstacles) -> list:
    if len(fw_obstacles) == 0: return []
    threshold_ttc = 3.5  # threshold_of time-to-colision

    target_obstacles = []
    for obstacles in fw_obstacles:
        obs_pos = obstacles['to_middle']

        if obstacles['dist'] <= 0: continue  # 지나간 장애물일 경우 무시
        # if abs(car_pos - obs_pos) >= 2.2: continue          # 차량 및 장애물의 거리차가 2.2m 이상일경우 무시
        ttc = obstacles['dist'] / ((car_speed / 3.6) + 0.001)  # 시속에 3.6을 나누어 m/s로 차원을 맞춤
        if ttc <= threshold_ttc: target_obstacles.append(obstacles)

    return target_obstacles


## 2.1 VFH를 활용한 전방 파악
def VFH_grid_map(car_speed, half_road_width, obstacles) -> list:
    grid_size = 0.1  # m
    num_cells = ceil((half_road_width * 2) / grid_size)

    grid_map = [0] * num_cells

    if not obstacles: return grid_map

    proximate_dist = obstacles[0]['dist']

    for obstacle in obstacles:
        cell_position = int((obstacle['to_middle'] + half_road_width) / grid_size)
        generalized = map_value(obstacle['dist'], proximate_dist, 200, 10, 0)
        if 0 <= cell_position < num_cells:
            grid_map[cell_position] += generalized
            for i in range(1, 10 + 5):  # 장애물의 좌우 폭을 고려, 그리드가 0.1m 이므로 1m씩 추가 및 여유 0.5m 추가
                if 0 <= cell_position - i < num_cells:
                    grid_map[cell_position - i] += generalized
                if 0 <= cell_position + i < num_cells:
                    grid_map[cell_position + i] += generalized

    return grid_map


## 2.2 장애물을 바탕으로 경로 분석
def path_planning(car_speed, car_pos, forward_map, half_road_width) -> int:
    num_cells = len(forward_map)
    target_path = [0] * num_cells

    grid_size = 0.1  # m
    car_position = int((car_pos + half_road_width) / grid_size)

    # 차량이 도로 내에 있을 경우
    if 0 <= car_position < num_cells:

        # if not forward_map[car_position] : return car_position # 장애물이 없을경우에는 정상주행

        for idx, point in enumerate(forward_map):

            if 1 <= point: continue  # 소수점 값 보정에 관한 문제 해결을 위해

            weight_of_obstacle = 1

            for i in range(1, 10 + 5):  # 차량의 폭을 고려, 그리드가 0.1m 이므로 1m씩 추가 및 여유 0.5m 추가
                if 0 <= point - i < num_cells and forward_map[point - i]:
                    weight_of_obstacle = 0.6
                    break
                if 0 <= point + i < num_cells:
                    weight_of_obstacle = 0.6
                    break

            closest_point_weight = calculate_weight(car_position, idx, num_cells)
            target_path[idx] = round(closest_point_weight * weight_of_obstacle, 2)

        target_way_points = get_max_pointed_path(target_path)

        ## 차량의 위치 기준
        if car_position <= half_road_width:  # 차량이 좌측이 위치할 경우
            target_point = max(target_way_points)  # 우측 경로로 주행

        if car_position > half_road_width:  # 차량이 우측에 위치할 경우
            target_point = min(target_way_points)  # 좌측 경로로 주행

        # if forward_map[car_position]:
        #     print("회피해야합니다,..")
        # print("경로 우선순위", target_path)
        return target_point
    else:
        # print("트랙을 벗어났습니다.")
        return 0 if car_position < 0 else num_cells - 1


def get_max_pointed_path(lst):
    max_value = max(lst)
    max_indexes = [i for i, value in enumerate(lst) if value == max_value]
    return max_indexes


def calculate_weight(car_position, idx, max_distance, max_score=100):
    distance = abs(car_position - idx)
    weight = int(max_score * (1 - distance / max_distance))
    return max(0, weight)


def generate_path(car_speed, car_pos, half_road_width, recommended_path, obstacles):
    grid_size = 0.1
    target_pos = (recommended_path + 1) * grid_size - half_road_width  # idx +1
    # print(f'target:{target_pos} now:{car_pos}')
    target_pos = target_pos * 1.1  # 여유값 제공

    if not obstacles: return 0
    # if abs(target_pos - car_pos) <= grid_size : return 0 # 직진의 경우에 대한 오차 보정

    proximate_dist = (car_speed / 3.6) + 0.001
    required_angle = _required_angle(proximate_dist, car_pos, target_pos)
    return required_angle


def _required_angle(proximate_dist, car_pos, target_pos):
    return max(min(atan2(proximate_dist, car_pos - (target_pos)) * (180 / pi) - 90, 50), -50)


def PI_controller(self):
    I_ax = 0.1  # 신호가 0.1초마다 업데이트
    E_ax = udp_input.DriveCont.Ax - car.ConBdy1.a_1[0]
    I_ax_integral += I_ax * E_ax

    # Clamp I_ax_integral to be between -1.0 and 1.0
    I_ax_integral = min(max(I_ax_integral, -1.0), 1.0)

    Cont_ax = P_ax * E_ax + I_ax_integral
    return Cont_ax


def map_value(value, min_value, max_value, min_result, max_result):
    '''maps value (or array of values) from one range to another'''
    if (max_value - min_value) == 0: return max_result
    result = min_result + (value - min_value) / (max_value - min_value) * (max_result - min_result)
    return result


if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)



