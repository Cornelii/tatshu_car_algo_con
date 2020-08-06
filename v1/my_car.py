from DrivingInterface.drive_controller import DrivingController
import numpy as np


class ToGoal:
    def __init__(self, is_debug=False):
        self.DUMMY = 99
        self.utils = Utils()
        self.is_debug = is_debug

    def preprocessing(self, sensing_info):
        data = sensing_info
        return data

    def modify_input(self, data, half_road_limit=7):
        origin_to_middle = data.to_middle
        origin_moving_angle = data.moving_angle

        # get obstacle_map
        # m = 5
        self.utils.get_obstacle_map(data, half_road_limit)

        # get to_middle to pass for obstacles
        to_middle_target, a, b = self.utils.get_optimal_pass_mid_point(data)

        actual_min_distance = self.utils.get_min_distance_to_obstacle(data, to_middle_target, a, b)

        if data.obstacle_flag:
            data.to_middle = np.dot(np.array([-0.8, 1.05, -0.001]), np.array([to_middle_target, origin_to_middle,
                                                                             origin_moving_angle]))
            data.moving_angle = np.dot(np.array([-0.8, -0.01, 1.0]), np.array([to_middle_target, origin_to_middle,
                                                                             origin_moving_angle]))

        if self.is_debug:
            print(data.obstacle_map)
            print(f"a: {a}, b: {b}")
            #print(f"to_middle_target: {to_middle_target}")
            print(f"obstacle_flag: {data.obstacle_flag}")
            #print(f"min_distance: {actual_min_distance}")

        return data

    def get_final_input(self, data):
        return data

    def set_steering(self, car_controls, data):
        car_controls.steering = data.moving_angle*-0.016 / (0.009*data.speed + 1)
        car_controls.steering += data.to_middle*-0.019 / (0.011*data.speed + 1)
        car_controls.steering += 0.008 * data.track_forward_angles[self.utils.get_significant_idx_for_steering(data.speed/3.6)]
        if data.obstacle_flag: # 0.008
            pass
        else:
            pass
            #car_controls.steering += 130 * data.kappa

    def set_throttle(self, car_controls, data):
        MIN_VELOCITY = 80
        SLOPE = - 1 / 700

        if data.speed <= MIN_VELOCITY:
            car_controls.throttle = 1
        else:
            car_controls.throttle = 1 + SLOPE * (data.speed - MIN_VELOCITY)
            if abs(data.moving_angle) > 10:
                car_controls.throttle = 0.8
            if data.track_forward_angles[9] < 10:
                car_controls.throttle += 1 / (20 + abs(data.track_forward_angles[9])*0.1)

    def set_brake(self, car_controls, data):
        if abs(data.track_forward_angles[9]) > 80:
            if data.speed > 110:
                car_controls.brake = 0.2
                car_controls.throttle = 0.8
            elif data.speed > 130:
                car_controls.brake = 0.4
            elif data.speed > 150:
                car_controls.brake = 0.8
        if abs(10 < data.track_forward_angles[2] < 20 and data.speed > 130):
            car_controls.brake = 0.3
            car_controls.throttle = 0.5

    def opponents_strategy(self, car_controls, data):
        pass


class Utils:

    def __init__(self):
        self.DUMMY = 99
        self.PI = 3.141592

    def get_obstacle_map(self, data, half_road_limit=7, m=None):
        # function to get obstacle map in the "data"
        # data => sensing_info - based object
        # half_road_limit => car_object.half_road_limit
        # m map-dividing factor map consists of 2*m list
        # data.obstacle List[]  No obstacle -> self.DUMMY. if there exists its to_middle value is included
        # data.delta = half_road_limit * 2 / (2 * m)

        if m:
            pass
        else:
            m = int(round(2*half_road_limit - 4)/2)
        n = 2 * m
        data.obstacle_map = [self.DUMMY] * n
        data.obstacle_flag = False

        forward_detection_range = data.speed / 3.6 * 1.68 + 15
        forward_undetected_range = data.speed / 3.6 * 0.1
        delta = 2 * half_road_limit / n

        data.delta = delta
        for obstacle in data.track_forward_obstacles:
            # make N partition on Road and get obstacle_map
            if forward_undetected_range <= obstacle["dist"] <= forward_detection_range:
                for i in range(n):
                    if data.obstacle_map[i] == self.DUMMY and obstacle["to_middle"] < delta * (i + 1) - half_road_limit:
                        data.obstacle_map[i] = obstacle["to_middle"]
                        if ~data.obstacle_flag:
                            data.obstacle_flag = True
                        break

    def get_optimal_pass_mid_point(self, data, n=None):
        # method to get kind of mid point of bypass to obstacle
        # data: sensing_info based object
        # n => length of obstacle map
        # return pass_mid_point, a, b (a: leftmost index of the widest range to pass, b: rightmost index)

        if data.obstacle_map:
            pass
        else:
            raise Exception("Get Obstacle map First. Use get_obstacle_map(data, half_road_limit)")
        if not n or len(data.obstacle_map) != n:
            n = len(data.obstacle_map)
        # get target to_middle_target
        i = 0
        a, b = 0, 0
        # point = 0
        while i < n:
            if data.obstacle_map[i] == self.DUMMY:
                # point = i
                for j in range(i + 1, n):
                    if data.obstacle_map[j] == self.DUMMY:
                        if j == n - 1 and j - i > b - a:
                            b = n - 1
                            a = i
                            i = j
                        continue
                    else:
                        if (j - i) > b - a:
                            a = i
                            b = j - 1
                        i = j
                        break
            i += 1
        to_middle_target = data.delta * (a + b + 1 - n) / 2
        return to_middle_target, a, b

    def get_min_distance_to_obstacle(self, data, to_middle_target, a, b, n = None):
        # function for actual minimum distacne between obstacle and optimal_pass_middle_point

        if data.obstacle_map:
            pass
        else:
            raise Exception("Get Obstacle map First. Use get_obstacle_map(data, half_road_limit)")
        if not n or len(data.obstacle_map) != n:
            n = len(data.obstacle_map)

        min_d_obstacle = data.delta
        if a > 0:
            min_d_obstacle = to_middle_target - data.obstacle_map[a-1]
        if b < n - 1 and data.obstacle_map[b+1] != self.DUMMY:
            if abs(min_d_obstacle) > abs(data.obstacle_map[b-1] - to_middle_target):
                min_d_obstacle = to_middle_target - data.obstacle_map[b-1]
        return min_d_obstacle

    def max(self, a, b):
        return a if a >= b else b

    def min(self, a, b):
        return a if a <= b else b

    def get_significant_idx_for_steering(self, speed):
        idx = int(self.min(9, (speed * 1.4)//10-3))
        if idx < 0:
            idx = 0
        return idx


class Escape:
    def __init__(self):
        self.valid = True
        self.MIN_D = 15
        self.collide_threshold = 20
        self.reverse_threshold = 20
        self.collide_count = 0
        self.reverse_count = 0
        self.stop_count = 0

    def is_valid(self, data, half_road_limit=7):
        if self.valid and not self.is_collided(data) and not self.is_reverse(data):
            return True
        else:
            return False

    def check_escaped(self, data, p_throttle, half_road_limit=7):
        if not data.collided:
            if self.get_abs_obstacle(data) >= self.MIN_D and self.is_escaped(data, p_throttle):
                self.valid = True
                self.collide_count = 0
                self.reverse_count = 0
                self.stop_count = 0

    def get_abs_obstacle(self, data):
        if data.track_forward_obstacles:
            return abs(data.track_forward_obstacles[0]["dist"])
        else:
            return self.MIN_D

    def is_collided(self, data):
        if data.collided:
            if self.valid:
                self.collide_count += 1
        return self.collide_count > self.collide_threshold

    def is_reverse(self, data):
        if not data.moving_forward:
            self.reverse_count += 1
        return self.reverse_count > self.reverse_threshold

    def is_escaped(self, data, p_throttle):
        if p_throttle > 0: # car_forward
            if data.moving_forward: # real_car_forward
                print("real_car_forward")
                if abs(data.moving_angle) < 70:
                    return True
            else:# real_car_backward
                print("real_car_backward")
                if data.moving_angle > 0:
                    data.moving_angle -= 170
                else:
                    data.moving_angle += 170
        else:
            if data.moving_forward: # real_car_backward
                print("real_car_backward")
                if data.moving_angle > 0:
                    data.moving_angle -= 170
                else:
                    data.moving_angle += 170
            else: # real_car_forward
                print("real_car_forward")
                if abs(data.moving_angle) < 70:
                    return True

        return False

    def set_values(self, data, car_controls):
        car_controls.throttle = -1
        car_controls.steering = data.moving_angle*0.015
        car_controls.steering += data.to_middle*0.02

        if data.speed == 0:
            self.stop_count += 1

        if self.stop_count > 20:
            self.valid = True


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #
        self.is_debug = True
        self.to_goal = ToGoal(self.is_debug)
        self.escape = Escape()
        self.p_throttle = 0
        self.p_steering = 0
        self.p_brake = 0

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

            print("[MyCar] collided: {}".format(sensing_info.collided))
            #print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            #print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            #print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################

        data = self.to_goal.preprocessing(sensing_info)
        self.escape.check_escaped(data, self.p_throttle, self.half_road_limit)

        if self.escape.is_valid(data, self.half_road_limit):
            print(f"1st to_middle: {data.to_middle} mv_angle: {data.moving_angle}")
            data = self.to_goal.modify_input(data, self.half_road_limit)

            print(f"2nd to_middle: {data.to_middle} mv_angle: {data.moving_angle}")

            data = self.to_goal.get_final_input(data)

            self.to_goal.set_steering(car_controls, data)
            self.to_goal.set_throttle(car_controls, data)
            self.to_goal.set_brake(car_controls, data)
        else:
            self.escape.set_values(data, car_controls)

        self.p_throttle = car_controls.throttle
        self.p_brake = car_controls.brake
        self.p_steering = car_controls.steering

        # Moving straight forward
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}"\
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


if __name__ == '__main__':
    print("[MyCar] Start Bot!")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot!")

    exit(return_code)
