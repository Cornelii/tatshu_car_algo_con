from DrivingInterface.drive_controller import DrivingController

DELTA_STEERING = 0.05
DELTA_THROTTLE = 0.1
DELTA_BRAKE = 0
MIN_SPEED = 50


class ToGoal:
    def __init__(self):
        pass

    def preprocessing(self, sensing_info):
        data = sensing_info
        return data

    def modify_input(self, data):
        return data

    def get_final_input(self, data):
        return data

    def set_steering(self, car_controls, data):
        if abs(data.to_middle) > 0.5:
            if data.to_middle > 0:
                car_controls.steering = -DELTA_STEERING
            else:
                car_controls.steering = DELTA_STEERING
        if abs(data.track_forward_angles[9] > 30):
            car_controls.steering *= data.track_forward_angles[9]/2

    def set_throttle(self, car_controls, data):
            car_controls.throttle = 0.5

    def set_brake(self, car_controls, data):
        if abs(data.track_forward_angles[9]) > 30 and data.speed > MIN_SPEED + 20:
            car_controls.brake = 0.2


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
        data = self.to_goal.modify_input(data)
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
