from DrivingInterface.drive_controller import DrivingController
import numpy as np

class ToGoal:
    def __init__(self):
        self.DUMMY = 99

    def preprocessing(self, sensing_info):
        data = sensing_info
        return data

    def modify_input(self, data, half_road_limit=7):
        origin_to_middle = data.to_middle
        origin_moving_angle = data.moving_angle
        M = 5
        N = 2 * M
        obstacle_list = [self.DUMMY]*N
        data.obstacle_flag = False

        d = data.speed / 3.6 * 1.5 + 5
        delta = 2 * (half_road_limit) / N
        print(delta)
        print(half_road_limit)

        for obstacle in data.track_forward_obstacles:
            # make N partition on Road and get obstacle_list
            if 2 <= obstacle["dist"] <= d:
                for i in range(N):
                    if obstacle_list[i] == self.DUMMY and obstacle["to_middle"] < delta * i - half_road_limit:
                        obstacle_list[i] = obstacle["to_middle"]
                        break
        # get target to_middle_target
        i = 0
        a, b = 0, 0
        point = 0
        while i < N:
            if obstacle_list[i] == self.DUMMY:
                point = i
                for j in range(i + 1, N):
                    if obstacle_list[j] == self.DUMMY:
                        if j == N-1:
                            b = N-1
                            a = point
                            i = j
                        continue
                    else:
                        if (j - 1 - i) > b - a:
                            a = i
                            b = j - 1
                        i = j
                        break

            i += 1
        print(obstacle_list)
        print(f"a:{a} b:{b}")
        to_middle_target = delta * (a + b + 1 - N) / 2
        min_d_obstacle = delta
        if a > 0:
            min_d_obstacle = abs(obstacle_list[a-1] - to_middle_target)
        if b < N - 1 and obstacle_list[b+1] != self.DUMMY:
            if min_d_obstacle > abs(obstacle_list[b-1] - to_middle_target):
                min_d_obstacle = abs(obstacle_list[b-1] - to_middle_target)

        data.to_middle = np.dot(np.array([-0.8, 1.0, -0.001]), np.array([to_middle_target, origin_to_middle,
                                                       origin_moving_angle]))
        data.moving_angle = np.dot(np.array([-0.8, -0.01, 1.0]), np.array([to_middle_target, origin_to_middle,
                                                          origin_moving_angle]))

        if sum(obstacle_list) != 990:
            data.obstacle_flag = True
        return data

    def get_final_input(self, data):
        return data

    def set_steering(self, car_controls, data):
        car_controls.steering = data.moving_angle*-0.012 / (0.007*data.speed + 1)
        car_controls.steering += data.to_middle*-0.02 / (0.01*data.speed + 1)
        car_controls.steering += 0.013*data.track_forward_angles[0]

    def set_throttle(self, car_controls, data):
        car_controls.throttle = 1
        if data.moving_angle > 10:
            car_controls.throttle = 0.7
        if (data.speed >= 150):
            car_controls.throttle = 0.8
        if (data.speed >= 200):
            car_controls.throttle = 0.3
        if abs(data.moving_angle) > 10:
            car_controls.throttle = 0.3

        if data.lap_progress < 0.5 or data.speed < 50:
            car_controls.throttle = 1

        if data.obstacle_flag:
            car_controls.throttle -= 0.1


    def set_brake(self, car_controls, data):
        if abs(data.track_forward_angles[9]) > 50:
            if data.speed > 80:
                car_controls.brake = 0.2
                car_controls.throttle = 0.7
            elif data.speed > 130:
                car_controls.brake = 0.5
            elif data.speed > 150:
                car_controls.brake = 0.9
        if abs(10 < data.track_forward_angles[2] < 20 and data.speed > 100):
            car_controls.brake = 0.3
            car_controls.throttle = 0.5

    def get_sign(self, x):
        return 1 if x >= 0 else -1

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.to_goal = ToGoal()
        self.is_debug = True

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
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################

        data = self.to_goal.preprocessing(sensing_info)

        print(f"1st to_middle: {data.to_middle} mv_angle: {data.moving_angle}")
        data = self.to_goal.modify_input(data, self.half_road_limit)

        print(f"2nd to_middle: {data.to_middle} mv_angle: {data.moving_angle}")

        data = self.to_goal.get_final_input(data)
        self.to_goal.set_steering(car_controls, data)
        self.to_goal.set_throttle(car_controls, data)
        self.to_goal.set_brake(car_controls, data)
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
