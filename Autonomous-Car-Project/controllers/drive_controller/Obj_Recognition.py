"""

    This script responsible of object recognition.

    Büşra Nur Bahadır 201511006

                                                    """
import time
from csv import QUOTE_ALL, writer
Log = list()
error_Log = list()
obj_data, LIDAR_data = None, None


def LIDAR_sensor(lidars):
    # TODO
    # left_obstacle = []
    # right_obstacle = []
    # ibeolux_width = lidars[0].getHorizontalResolution()
    # max_range = ibeolux_width.getMaxRange()
    # range_threshold = max_range / 20.0
    # size = ibeolux_width * lidars[0].getNumberOfLayer
    # for x in lidars:
    #     lidar_values = x.getLayerRangeImage()
    #     for i in range(0, ibeolux_width, 1):
    #         if lidar_values[i] < range_threshold:  # far obstacles are ignored
    #             if "right" in key:
    #             left_obstacle += size[i] * (1.0 - lidar_values[i] / max_range)
    # print(lidars)
    return None


def dist_sensor(dist_sensors, dist_sensor_names):
    """
        To find near objects with distance sensors
           :param dist_sensor_names: name of the distance sensors for indexing
           :param dist_sensors: distance sensors
           :return dictionary which contains the distance sensor names and values of the distance
           :rtype dict

    """
    obj_on_side = {}
    for j in range(len(dist_sensor_names)):
        if dist_sensors[dist_sensor_names[j]].getValue() < dist_sensors[dist_sensor_names[j]].getMaxValue():
            obj_on_side.update({dist_sensor_names[j]: dist_sensors[dist_sensor_names[j]].getValue()})
        else:
            obj_on_side.pop(dist_sensor_names[j], None)
    return obj_on_side


def cam_obj_rec(camera, string):
    """
        To find near objects with camera sensors
           :param string: str for print
           :param camera: Camera sensor

    """
    obj_to_reduce = {"road", "building", "tree", "hotel"}
    cam_objects = camera.getRecognitionObjects()

    for i in range(0, len(cam_objects)):
        s1 = set(cam_objects[i].model.split())
        if not bool(s1.intersection(obj_to_reduce)):
            # may write to Logs later
            print("Model of object at " + string + "{} : {}".format(i, cam_objects[i].model))
            print(cam_objects[i].get_position_on_image())
            print(cam_objects[i].get_position())
            print(cam_objects[i].get_size_on_image())


def main(dist_sensor_names, lidars, dist_sensors, front_cams, back_camera):
    """
           :param back_camera:
           :param front_cams:
           :param dist_sensors:
           :param lidars:
           :param dist_sensor_names:
    """
    global obj_data, LIDAR_data
    start_time = time.time()
    Log.clear()
    error_Log.clear()
    try:
        obj_data = dist_sensor(dist_sensors, dist_sensor_names)
        LIDAR_data = None
        # cam_obj_rec(front_camera1, "front")
        # cam_obj_rec(back_cam, "rear")
        Log.append(str(time.time() - start_time))
    except Exception as e:
        error_Log.append("[OBJ] IN OBJ MAIN: %s " % e)
    with open("Logs\OBJ_Log.csv", 'a') as file:
        wr = writer(file, quoting=QUOTE_ALL)
        wr.writerow(Log)
    if len(error_Log):
        with open("Logs\error_Log.csv", 'a', newline="") as file:
            wr = writer(file, quoting=QUOTE_ALL)
            wr.writerow(error_Log)
    return obj_data, LIDAR_data


if __name__ == '__main__':
    main()
