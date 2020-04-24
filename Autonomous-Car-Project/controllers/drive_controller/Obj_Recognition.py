"""

    This script responsible of object recognition.

    Büşra Nur Bahadır 201511006

                                                    """
import math

"""------------------------------------------------------------------------------------------------------------"""


def LIDAR_sensor(lidars):
    """ Ill write necessarily algorithms later"""
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
    res = 0
    return res


"""------------------------------------------------------------------------------------------------------------"""
# distance sensors
dist_sensor_names = [
    "front",
    "front right 0",
    "front right 1",
    "front right 2",
    "front left 0",
    "front left 1",
    "front left 2",
    "rear",
    "rear left",
    "rear right",
    "right",
    "left"]


# to find near objects with distance sensors
def dist_sensor():
    obj_on_side = {}
    for j in range(len(dist_sensor_names)):
        if dist_sensors[dist_sensor_names[j]].getValue() < dist_sensors[dist_sensor_names[j]].getMaxValue():
            obj_on_side.update({dist_sensor_names[j]: dist_sensors[dist_sensor_names[j]].getValue()})
        else:
            obj_on_side.pop(dist_sensor_names[j], None)
    return obj_on_side


"""------------------------------------------------------------------------------------------------------------"""

obj_to_reduce = set(["road", "building", "tree", "hotel"])


def cam_obj_rec(camera, string):
    cam_objects = camera.getRecognitionObjects()

    for i in range(0, len(cam_objects)):
        s1 = set(cam_objects[i].model.split())
        if not bool(s1.intersection(obj_to_reduce)):
            print("Model of object at " + string + "{} : {}".format(i, cam_objects[i].model))
    pass


"""------------------------------------------------------------------------------------------------------------"""


def main(m_lidars, m_dist_sensors, m_front_cam, m_back_cam):
    global lidars, dist_sensors, front_cam, back_cam
    lidars = m_lidars
    dist_sensors = m_dist_sensors
    obj_data = dist_sensor()
    LIDAR_data = LIDAR_sensor(lidars)
    front_cam, back_cam = m_front_cam, m_back_cam
    # cam_obj_rec(front_cam, "front")
    # cam_obj_rec(back_cam, "rear")
    return obj_data, LIDAR_data


if __name__ == '__main__':
    main()
