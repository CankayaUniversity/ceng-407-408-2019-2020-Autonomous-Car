"""
    This script responsible of data fusion.

    Büşra Nur Bahadır 201511006

                                                                        """
import math
import PID_control
import Storage
import lane_management

"""------------------------------------------------------------------------------------------------------------"""


# To change lane center according to distance sensor data
def obj_center(dist_sensors, dist_sensor_data):
    for key, value in dist_sensor_data.items():
        if value < (dist_sensors[key].getMaxValue()/2):
            s = key.split()
            res = round(dist_sensors[key].getMaxValue() - value) * 2
            if "right" in key:
                return -res
            if "left" in key:
                return +res
    else:
        return 0


"""------------------------------------------------------------------------------------------------------------"""


# check sides of the car for lane changing
def check_sides(dist_sensor_data):
    is_available_left = obj_checker(dist_sensor_data, "left")
    is_available_right = obj_checker(dist_sensor_data, "right")
    if is_available_left:
        return "left"
    elif is_available_right:
        return "right"
    else:
        return None


"""------------------------------------------------------------------------------------------------------------"""


# call lane_management with side variable to change lane
def change_lane(display_front, front_camera, auto_drive, gps, status, side):
    if side is not None:
        cl = lane_management.main(display_front, front_camera, auto_drive, gps, side)
        return cl
    else:
        if status == "front":
            while auto_drive.getCurrentSpeed() > 0:
                auto_drive.setCruisingSpeed(0)
        elif status == "emergency":
            auto_drive.setCruisingSpeed(auto_drive.getCurrentSpeed() + 30)
        return gps


"""------------------------------------------------------------------------------------------------------------"""


# check if there is a distance sensor variable comes from object recognition
def obj_checker(dist_sensor_data, string):
    flag = 0
    if string == "front":
        if string in dist_sensor_data:
            flag = 1
        else:
            if flag == 0:
                return True
            else:
                return False

    for key, value in dist_sensor_data.items():
        s = key.split()
        if string in s:
            flag = 1
    else:
        if flag == 0:
            return True
        else:
            return False


"""------------------------------------------------------------------------------------------------------------"""


# main function to call necessarily parts
def main(auto_drive, gps, dist_sensor_data, lidar_data, emergency, display_front, front_camera,dist_sensors):
    if "rear" in dist_sensor_data:
        auto_drive.setCruisingSpeed(auto_drive.getCurrentSpeed()+10)
    is_available_front = obj_checker(dist_sensor_data, "front")
    change_side = Storage.loadData("change_side", "Lane")
    if not is_available_front:
        if change_side is None:
            change_side = check_sides(dist_sensor_data)
            Storage.storeData("change_side", change_side, "Lane")
        cl = change_lane(display_front, front_camera, auto_drive, gps, "front", change_side)
    if emergency:
        if change_side is None:
            change_side = check_sides(dist_sensor_data)
            Storage.storeData("change_side", change_side, "Lane")
        cl = change_lane(display_front, front_camera, auto_drive, gps, "emergency", change_side)
    if not emergency and is_available_front:
        cl = lane_management.main(display_front, front_camera, auto_drive, gps, None)
        if change_side is not None:
            Storage.storeData("change_side", None, "Lane")
    if cl is not None and not math.isnan(cl):
        cl = cl + obj_center(dist_sensors, dist_sensor_data)
        PID_control.main(auto_drive, gps, cl)


if __name__ == '__main__':
    main()
