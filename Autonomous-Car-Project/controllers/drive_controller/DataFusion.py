"""
    This script responsible of data fusion.

    Büşra Nur Bahadır 201511006

                                                                        """


def main(gps, lane_man_data, dist_sensor_data, lidar_data, emergency):
    """ I'll write necessarily algorithms to find value for sending to PID later
    for now its only works with lane management data """
    # res = 0
    # for key, value in dist_sensor_data.items():
    #     s = key.split()
    #     if "front right" in key:
    #         res = gps - round((20-dist_sensor_data[key])*(3-int(s[2])))
    #     if "front" in key and "left" not in key:
    #         res = gps + round((20-dist_sensor_data[key]))
    #     if "front left" in key:
    #         res = gps + round((20-dist_sensor_data[key])*(3-int(s[2])))
    # if lane_man_data is not None and res != 0:
    #     print("gps",gps)
    #     print("res",res)
    #     print("lane",lane_man_data)
    #     res = round((res + lane_man_data) / 2, 2)
    # else:
    res = lane_man_data

    return res


if __name__ == '__main__':
    main()
