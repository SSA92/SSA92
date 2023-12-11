from DrivingInterface.drive_controller import DrivingController

from math import atan2, asin, tan, pi, cos, sin, ceil, sqrt

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
        
        self.is_collided = False
        self.collided_counter = 10
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
        angle_num = int(sensing_info.speed / 45)
        ref_angle = sensing_info.track_forward_angles[angle_num] if angle_num > 0 else 0
        ref_distance = sensing_info.distance_to_way_points[angle_num] if angle_num > 0 else 0
        
        
        ## 0.1 throttle 값
        if sensing_info.speed < 60: set_throttle = 1  
        else : set_throttle =  min( map_value(sensing_info.speed,0,60,0,1), 1)
        
        ## 0.2 break 값
        set_brake = 0
        
        ## 0.3. 차량의 Speed 에 따라서 핸들을 돌리는 값[steer_factor]
        steer_factor = sensing_info.speed * 1.5
        if sensing_info.speed > 70: steer_factor = sensing_info.speed * 0.9
        if sensing_info.speed > 100: steer_factor = sensing_info.speed * 0.75
        if sensing_info.speed > 150: steer_factor = sensing_info.speed * 0.66
        if sensing_info.speed > 170: steer_factor = sensing_info.speed * 0.5
        
        ## 0.4 main route로 가기위한 steering angle 설정
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)
        ## 0.5 충돌 여부 파악
        if sensing_info.collided : self.is_collided = True

        
        ## 2. 장애물에 따른 장애물 극복 로직
        ### 2.1 장애물 파악
        objects = analyze_obstacles(sensing_info.speed, sensing_info.to_middle, sensing_info.track_forward_obstacles, 
                                    sensing_info.opponent_cars_info)
        
        if not self.is_collided and (abs(ref_angle) >= 4 ): 
            # 충돌 시 안정적인 장애물 회피를 위해 장애물의 위치에 대한 계산을 하지 않음
            # 직선 주행과 가까울 경우 계산을 하지 않는 것이 더욱 안정적인 주행을 하는 것을 실험적으로 확인
            objects = calculate_obstacles(sensing_info.to_middle, sensing_info.track_forward_angles, sensing_info.distance_to_way_points, objects)
        
        if self.is_collided : 
            self.collided_counter -= 1
            if self.collided_counter <= 0 : self.is_collided = False

        ### 2.2 전방 파악
        mapped_data = VFH_grid_map(sensing_info.speed, half_load_width, objects)
        
        ### 2.3 전방 데이터를 바탕으로 경로 설정
        path_recommend = path_planning(sensing_info.speed, sensing_info.moving_angle, sensing_info.to_middle, mapped_data, half_load_width, sensing_info.track_forward_obstacles)
        
        ### 2.4 해당 경로를 바탕으로 회피각도 설정
        avoidance_angle = generate_path(sensing_info.speed, sensing_info.to_middle, half_load_width, path_recommend, objects) 

        ## 2.5 장애물 회피 각도 제공
        if avoidance_angle:
            selcted_path = 1.25 * min(1, sensing_info.speed/10) * ( map_value(abs(avoidance_angle),0,50,0,1) + 1)* (avoidance_angle) / (steer_factor + 0.001)
            set_steering += selcted_path
        
        gen_ref_angle = map_value(abs(ref_angle),0,50,0,1) + 0.55
        ref_mid = (sensing_info.to_middle /(sensing_info.speed+0.001)) * -1.2
        middle_add = gen_ref_angle * ref_mid
        if not objects or abs(self.half_road_limit) <= abs(sensing_info.to_middle):
            set_steering += middle_add



        # Moving straight forward
        car_controls.steering = set_steering
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake

        ################################################################################################################

        # Moving straight forward

            
        # car_controls.steering = PI_controller(sensing_info.moving_angle, set_steering, steer_factor)
        car_controls.steering = set_steering
        # print(f'{sensing_info.moving_angle} {set_steering * (steer_factor+ 0.001)}')
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

# Vector Field Histogram 에 대한 설명 : https://kr.mathworks.com/help/nav/ug/vector-field-histograms.html
grid_size = 0.05 # VFH 매핑을 위한 Grid 값 설정.

## 장애물은 모든 맵 기준 2m 이며 to_middle 기준 좌우 1m입니다. 
## 1. 장애물을 분석
def analyze_obstacles(car_speed, car_pos, fw_obstacles, fw_opponent_cars) -> list:
    if len(fw_obstacles) == 0 or len(fw_opponent_cars): 
        return []
    threshold_ttc = 4 # threshold_of time-to-colision 충돌 안전 s를 4초로 설정
    
    target_obstacles = []
    for obstacles in fw_obstacles:
        obs_pos = obstacles['to_middle']
        
        if obstacles['dist'] <= 0 : continue                # 지나간 장애물일 경우 무시
        ttc = obstacles['dist']/((car_speed /3.6)+ 0.001)   # 시속에 3.6을 나누어 m/s로 차원을 맞춤
        if ttc <= threshold_ttc: target_obstacles.append(obstacles) # ttc가 4미만일 경우에 충돌을 고려함.
    
    for obstacles in fw_opponent_cars:
        obs_pos = obstacles['to_middle']
        
        if obstacles['dist'] <= 0 : continue                # 지나간 장애물일 경우 무시
        ttc = obstacles['dist']/((car_speed /3.6)+ 0.001)   # 시속에 3.6을 나누어 m/s로 차원을 맞춤
        if ttc <= threshold_ttc: target_obstacles.append(obstacles) # ttc가 4미만일 경우에 충돌을 고려함.
    
    return target_obstacles

def calculate_obstacles(to_middle, track_forward_angles, distance_to_way_points, track_forward_obstacles):
    def rad_targAngle(C, i):
        nonlocal angles, points
        temp = points[i] * sin(C * pi / 180) / points[i+1]
        return asin(temp if abs(temp) <= 1 else int(temp)) * 180 / pi
    """    
    Parameters:
        to_middle: 중앙선으로부터의 거리
        track_forward_angles: 앞쪽 각도 정보 리스트
        distance_to_way_points: 웨이포인트까지의 거리 정보 리스트
        track_forward_obstacles: 앞쪽 장애물 정보 리스트
    
    Returns:
        obs: 장애물 좌표 리스트
    """
    
    # 오른쪽인가 왼쪽인가
    plag = 1 if to_middle >= 0 else -1
    points = [to_middle*plag, ] + distance_to_way_points
    angles = [track_forward_angles[0]*plag,] + [angle*plag for angle in track_forward_angles]


    bo = [(angles[1] - angles[0]), ]
    ts = []
    for i in range(20):
        C = 180 - bo[i] - (angles[i+1] - angles[i])
        A = rad_targAngle(C, i)
        bo.append(A)
        ts.append(180 - C - A)
    
    # 장애물 좌표
    obs = []
    near = abs(points[0] * cos((90 - angles[1]) * pi / 180)) + points[1] * cos(bo[1] * pi / 180)
    for obj in track_forward_obstacles:
        d, m = obj['dist'] - near, obj['to_middle']

        n, k = int(d // 10), d % 10
        ang = (90 - angles[n+1] * plag) * pi / 180
        
        obs.append({
            'dist' : points[n+1] * sin(sum(ts[:n+1]) * pi / 180) + k * sin(ang) - m * cos(ang),
            'to_middle' : - points[n+1] * plag * cos(sum(ts[:n+1]) * pi / 180) + k * cos(ang) + m * sin(ang)
            })

    return obs

## 2.1 VFH를 활용한 전방 파악
def VFH_grid_map(car_speed, half_road_width, obstacles) -> list:
    global grid_size
    num_cells = ceil((half_road_width*2) / grid_size)
    grid_map = [0] * num_cells
    
    if not obstacles : return grid_map
    
    min_C = obstacles[0]['dist']
    min_B = obstacles[0]['to_middle']
    min_A = sqrt(abs(min_C**2 - min_B**2))
    # 가장 가까운 장애물을 바탕으로 장애물을 히스토그램으로 표현
    
    for obstacle in obstacles:
        cell_position = int((obstacle['to_middle'] + half_road_width) / grid_size)
        A = sqrt(abs(obstacle['to_middle']**2 - obstacle['dist']**2))
        generalized = map_value(A, min_A, 200, 10, 0) # 가장 가까운 장애물에 대한 최대점수 가중치 부여
        if 0 <= cell_position < num_cells:
            grid_map[cell_position] += generalized
            for i in range(1,int(1/grid_size * 0.7)):  # 장애물의 좌우 폭을 고려, 그리드가 0.05m 이므로 1m씩 추가 및 여유 0.4m 추가
                if 0 <= cell_position - i < num_cells:
                    grid_map[cell_position - i] += generalized
                if 0 <= cell_position + i < num_cells:
                    grid_map[cell_position + i] += generalized
    return grid_map


## 2.2 장애물을 바탕으로 경로 분석
def path_planning(car_speed, car_yaw, car_pos, forward_map, half_road_width, obstacles) -> int:
    global grid_size
    # grid_size = 0.1 # m
    num_cells = len(forward_map)
    target_path = [0]*num_cells

    car_position = int((car_pos + half_road_width) / grid_size)
    proximate_dist = ((car_speed /3.6)) *4  # m/0.1s
    min_obj_dist = 200 if not obstacles else obstacles[0]['dist']
    # print(f'가까운 물체 : {min_obj_dist} 예상 이동거리 :{proximate_dist}')
    # 차량이 도로 내에 있을 경우
    if 0<= car_position < num_cells:
        
        for idx, point in enumerate(forward_map):
            if point >= 9.9 : continue # 장애물이 있는 지점은 무시 0 점
            
            
            # 자동차의 진행방향과, 현재지점과 목표지점과 각도의 유사성에 대한 가중치
            target_pos = _map2pos(idx, car_pos, grid_size, half_road_width)
            target_angle = _required_angle(min_obj_dist, car_pos, target_pos)
            weight_of_angle = calculate_weight(car_yaw, target_angle, 50, 1)
            score_closest_angle = calculate_weight(car_yaw, target_angle, 50, max_score=34)

            
            # 차량 주행시 충돌 안전성에 관한 가중치
            # 현재의 위치에서 자동차의 진행 방향으로 계속 갔을 경우 충돌이 있는지 계산해야함
            weight_of_obstacle = calculate_weight(forward_map[idx], 0, 10, 1) 
            score_closest_point = calculate_weight(car_position, idx, num_cells, max_score=66) 
            
            for i in range(1,int(1/grid_size * 0.7)):  # 장애물의 좌우 폭을 고려, 그리드가 0.05m 이므로 1m씩 추가 및 여유 0.4m 추가
            # 해당 지점에 히스토그램에 값이 ''10 주변인 장애물이 있을경우''[가장 가까운 장애물], 가중치를 최소화
                if (0 <= idx - i < num_cells and forward_map[idx - i] <= 10.1):
                    weight_of_obstacle = 0.3
                    break
                if (0 <= idx + i < num_cells and forward_map[idx + i] <= 10.1):
                    weight_of_obstacle = 0.3
                    break
            
            # 해당 방향으로 이동 시 충돌이 예상될 경우, 해당 위치에 대한 가중치를 줄임
            if (proximate_dist) > sqrt(((car_position - idx)* 0.1)**2 + min_obj_dist**2):
                weight_of_angle = 0.1
            
            
            ## 직선주행 중일 경우, 장애물에 대한 가중치를 높게봄
            if abs(car_yaw) < 5:  
                weight_of_obstacle *= 1.8

            target_path[idx] = round(score_closest_point * weight_of_obstacle + weight_of_angle * score_closest_angle, 2)
        
        # 가장 가중치가 높은 지점을 추출
        target_way_points = get_max_pointed_path(target_path)
        ## 차량의 위치 기준으로 경로 세분화 
        if car_position <= half_road_width : # 차량이 좌측이 위치할 경우
            target_point = max(target_way_points) # 우측 경로로 주행 

        if car_position > half_road_width : # 차량이 우측에 위치할 경우
            target_point = min(target_way_points) # 좌측 경로로 주행
        # print(forward_map)
        # print(target_path)
        return target_point
    else:
        # print("트랙을 벗어났습니다.")
        return -1

def get_max_pointed_path(lst):
    max_value = max(lst)
    max_indexes = [i for i, value in enumerate(lst) if value == max_value]
    return max_indexes

def calculate_weight(car_inform, target_inform, total, max_score=100):
    diff = abs(car_inform - target_inform)
    weight = round(max_score * (1 - diff/total), 1) 
    return max(0, weight)

def generate_path(car_speed, car_pos, half_road_width, recommended_path, obstacles):
    global grid_size
    target_pos = _map2pos(recommended_path, car_pos, grid_size, half_road_width)
    if recommended_path == -1 : return
    if not obstacles : return 0    
    proximate_dist = (car_speed /3.6) # 자동차 주행 속도를 바탕으로 angle값 결정    
    required_angle = _required_angle(proximate_dist, car_pos, (target_pos))
    return required_angle 

def _map2pos(recommended_path, car_pos, grid_size, half_road_width):
    target_pos = ((recommended_path+1 ) * grid_size - half_road_width) # (idx +1) * 그리드 - width = to_middle inform
    return target_pos + 0.15 if car_pos <= target_pos else target_pos - 0.15 # 여유값 제공

def _required_angle(proximate_dist, car_pos, target_pos):
    target_angle = (atan2(proximate_dist, (car_pos - target_pos)) * (180/pi) - 90 )
    # if abs(car_pos - target_pos) <= 0.2:
    #     return  target_angle + 0.3 if target_angle >= 0  else target_angle - 0.3
    return  target_angle

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


