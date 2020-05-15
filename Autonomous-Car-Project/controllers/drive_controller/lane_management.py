"""

    This script responsible of image processing for detecting lines, checking if neighbour lane exists.

    Büşra Nur Bahadır 201511006

                                                                                                            """
import time
from typing import Any, Union, Optional
import cv2
import numpy as np
from controller import Display
from math import sqrt
from csv import writer, QUOTE_ALL
import Storage

"""----------------------------------------------------------"""
Log = list()
error_Log = list()
E = 80  # a constant value to stay this much far from lane (only used when we can detect one line)
MIN_SIZE_REDUCE = 4
height = 0
width = 0
has_right_lane = False
has_left_lane = False
center_of_lane_res = None
region_of_interest_vertices = []
"""----------------------------------------------------------"""
right_lines = np.empty((0, 4), dtype=np.uint8)
left_lines = np.empty((0, 4), dtype=np.uint8)
reduced_lines = np.empty((0, 4), dtype=np.uint8)
"""----------------------------------------------------------"""
# constants for HoughPLines
rho = 1  # distance resolution in pixels
theta = np.pi / 180  # angular resolution in radians
threshold = 5  # minimum number of pixels
min_line_length = 10  # minimum distance between lines
max_line_gap = 30  # maximum gap between line segments
"""----------------------------------------------------------"""


def region_of_interest(image, vertices):
    """
     For masking other parts of the image than our interest
        :param vertices: vertices of out interest
        :param image: image
        :return masked image
     """
    global height, width
    # blank matrix with same h,w with the image
    mask = np.zeros((height, width), dtype=np.uint8)
    # mask everything other than region of interest
    cv2.fillPoly(mask, vertices, 255)
    # return the image
    res = cv2.bitwise_and(image, image, mask=mask)
    return res


def ascending_sort(line_arr):
    res = np.array(sorted(line_arr, key=lambda x: x[0]))
    return res


def descending_sort(line_arr):
    res = np.array(sorted(line_arr, key=lambda x: -x[0]))
    return res


def reduce_lines(line_arr, side):
    """
      For reducing lines found by Hough Probabilistic Lines and separated by left_right_finder()
         :param side: side of the lines
         :param line_arr: numpy array of lines
         :return reduced numpy array of lines by sides
      """
    start_time = time.time()
    try:
        if side == "right":
            line_arr = descending_sort(line_arr)
        elif side == "left":
            line_arr = ascending_sort(line_arr)

        lst = list()
        x = 5
        for i in range(0, len(line_arr) - 1):
            if any(abs(line_arr[i] - line_arr[i + 1])) < x:
                lst.append(i)
            else:
                pass
        line_arr = np.delete(line_arr, lst, axis=0)
        Log.append(str(time.time() - start_time))
        return line_arr
    except Exception as e:
        error_Log.append("[LANE MANAGEMENT] error reduce lines%s" % e)


def right_left_finder(lines):
    """
          For deciding which line belongs to which directions with slope
             :param lines: numpy array of lines found by Hough Probabilistic Lines
             :return numpy array of total reduced lines
    """
    start_time = time.time()
    global right_lines, left_lines, reduced_lines, has_right_lane, has_left_lane
    try:
        for n in range(0, len(lines)):
            if not lines[n][0][2] - lines[n][0][0]:  # avoiding division by 0
                pass
            elif not lines[n][0][3] - lines[n][0][1]:  # horizontal line
                pass
            else:
                slope = (lines[n][0][3] - lines[n][0][1]) / (lines[n][0][2] - lines[n][0][0])
                if 0.34 < abs(slope) < 0.9:
                    if slope > 0 and lines[n][0][0] > (width / 2) + 10 and lines[n][0][2] > (width / 2) + 10:
                        right_lines = np.append(right_lines, np.array([lines[n][0]]), axis=0)
                    elif slope < 0 and lines[n][0][0] < (width / 2) - 10 and lines[n][0][2] < (width / 2) - 10:
                        left_lines = np.append(left_lines, np.array([lines[n][0]]), axis=0)
        if len(right_lines) > MIN_SIZE_REDUCE:
            right_lines = reduce_lines(right_lines, "right")
        else:
            Log.append("smaller than MIN_SIZE_REDUCE")
        if len(left_lines) > MIN_SIZE_REDUCE:
            left_lines = reduce_lines(left_lines, "left")
        else:
            Log.append("smaller than MIN_SIZE_REDUCE")

        if len(right_lines):
            reduced_lines = np.append(reduced_lines, np.array(right_lines), axis=0)
        if len(left_lines):
            reduced_lines = np.append(reduced_lines, np.array(left_lines), axis=0)

        if len(right_lines) > 1:
            has_right_lane = True
        else:
            has_right_lane = False
        if len(left_lines) > 1:
            has_left_lane = True
        else:
            has_left_lane = False
        Log.append(str(time.time() - start_time))
    except Exception as e:
        error_Log.append("[LANE MANAGEMENT] error in left-right:%s" % e)


def toGPS_val(val, gps):
    """
          For find image point as a gps value (target on display)*gps/( gps's value on display)
             :param gps:
             :param val: point value in image
             :return image point's gps value
             :rtype float
    """
    global width, height
    res = (val * gps) / (width / 2)
    # res_x = ((val - (width / 2)) * (height / width))
    # res_z_2 = abs(res ** 2 - res_x ** 2)
    # res = sqrt(res_x ** 2 + res_z_2)
    return res


def center_of_lane(lane_change, gps):
    global left_lines, right_lines
    global width, height, region_of_interest_vertices
    start_time = time.time()
    center_of_the_lane_res = gps
    """------------------------------------------------------------------------------------------------------------"""
    try:
        # switch lane directions to change lane
        if lane_change == "left" and len(left_lines):
            print("[LANE MANAGEMENT] ----------------------------lane change to left..")
            right_lines = left_lines
            left_lines = np.empty((0, 4), dtype=np.uint8)
        elif lane_change == "right" and len(right_lines):
            print("[LANE MANAGEMENT] ----------------------------lane change to right..")
            left_lines = right_lines
            right_lines = np.empty((0, 4), dtype=np.uint8)
        """---------------------------------------------------------------------------------------------------------"""
        left, right = None, None
        # decide center of line
        if len(right_lines):
            r_mean = np.mean(right_lines, axis=0)
            right = round((r_mean[2] + r_mean[0]) / 2, 2)
        if len(left_lines):
            l_mean = np.mean(left_lines, axis=0)
            left = round((l_mean[2] + l_mean[0]) / 2, 2)

        try:
            if right is None and left is not None:
                center_of_the_lane_res = toGPS_val(left + E, gps)
            elif right is not None and left is None:
                center_of_the_lane_res = toGPS_val(right - E, gps)
            elif right is not None and left is not None:
                center_of_the_lane_res = toGPS_val((right - left) / 2 + left, gps)
        except ValueError:
            center_of_the_lane_res = gps
            error_Log.append("[LANE MANAGEMENT] error an exception occurred..")
        Log.append(str(time.time() - start_time))
        return round(center_of_the_lane_res, 2)
    except Exception as e:
        error_Log.append("[LANE MANAGEMENT] error center_of_lane %s" % e)


def draw_and_display_lines(display_front, image_n_arr, lines):
    """
          For finding center of lanes and Draw lines on the original image
             :param image_n_arr: numy arr of image from front camera
             :param display_front:
             :param lines: numpy array of total reduced lines
    """
    try:
        start_time = time.time()
        # draw center of line to display screen
        blank_img = np.zeros((image_n_arr.shape[0], image_n_arr.shape[1], image_n_arr.shape[2]), dtype=np.uint8)
        """----------------------------------------------------------------------------------------------------------"""
        for x in range(0, len(lines)):
            pts = np.array([[lines[x][0], lines[x][1]], [lines[x][2], lines[x][3]]], np.int32)
            cv2.polylines(blank_img, [pts], True, (0, 255, 0), thickness=5)
        """--------------------------------------------------------------------------------------------------------- """
        # Merge two images
        blank_img = cv2.addWeighted(image_n_arr, 0.8, blank_img, 1, 0.0)
        # cv2.imshow("center",image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        if blank_img is not None:
            # convert numpy array for display
            try:
                blank_img = np.array(np.swapaxes(blank_img, 0, 1)).tolist()
                # print(type(img))
                # print(img)
                img_ref = display_front.imageNew(blank_img, Display.BGRA, height=512, width=256)
                # blend for opacity
                display_front.imagePaste(img_ref, 0, 0, blend=False)
                if img_ref is not None:
                    display_front.imageDelete(img_ref)
                else:
                    error_Log.append("[LANE MANAGEMENT] error img ref none..")
            except ValueError:
                error_Log.append("[LANE MANAGEMENT] value error on display screen..")
        Log.append(str(time.time() - start_time))
    except Exception as e:
        error_Log.append("[LANE MANAGEMENT] error in draw lines:%s" % e)


def image_processing(image_np):
    """
            Main part of image processing for lane detection
                :param image_np: np array of image from front camera
                :return Hough Lines
       """
    start_time = time.time()
    global height, width
    try:
        # CV2
        # image_np = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 4)
        # print(type(np_arr))
        # print(np_arr.shape)
        gray_img = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        # if image is all black no need for processing
        if cv2.countNonZero(gray_img) == 0:
            error_Log.append("[LANE MANAGEMENT] error couldn't take image data...")
            return None
        # create hsv
        hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
        # crop image for interest region
        hsv = region_of_interest(hsv, np.array([region_of_interest_vertices], np.int32))
        # set color limits
        lower_val = (0, 0, 0)
        upper_val = (179, 45, 96)
        # threshold
        mask = cv2.inRange(hsv, lower_val, upper_val)
        # remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((8, 8), dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((20, 20), dtype=np.uint8))
        # improve mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            hull = cv2.convexHull(cnt)
            cv2.drawContours(mask, [hull], 0, 255, -1)
        # erode mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel=np.ones((5, 5), dtype=np.uint8))
        # apply mask
        road_hsv = cv2.bitwise_and(hsv, hsv, mask=mask)
        # set color limits
        lower_val = (0, 0, 102)
        upper_val = (179, 255, 255)
        # Threshold the HSV image
        mask2 = cv2.inRange(road_hsv, lower_val, upper_val)
        # apply mask to original image
        result = cv2.bitwise_and(image_np, image_np, mask=mask2)
        result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        lines = cv2.HoughLinesP(result, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

        # cv2.imshow("Result", result)
        # cv2.imshow("Image", image_np)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        Log.append(str(time.time() - start_time))
        if lines is not None:
            return lines
        else:
            error_Log.append("[LANE MANAGEMENT] error lines couldn't find by Hough Lines..")
            return None
    except Exception as e:
        error_Log.append("[LANE MANAGEMENT] error in image processing...%s" % e)


def main(display_front: object, camera_instance: vars, m_gps: float, m_lane_change: vars, flag: bool) -> \
        Union[Optional[int], Any]:
    start_time = time.time()
    """
          main to call other functions
              :rtype: object
              :param flag: flag for displaying or not
              :param camera_instance: front camera
              :param m_lane_change: status of lane changing if its not None need to change lane
              :param m_gps: gps value
              :param display_front: display screen
              :return: center of the lane as gps value or  original gps value without changing
    """
    global right_lines, left_lines, reduced_lines, center_of_lane_res, \
        height, width, region_of_interest_vertices
    Log.clear()
    error_Log.clear()
    right_lines = np.empty((0, 4), dtype=np.uint8)
    left_lines = np.empty((0, 4), dtype=np.uint8)
    reduced_lines = np.empty((0, 4), dtype=np.uint8)
    height = camera_instance.getHeight()
    width = camera_instance.getWidth()
    region_of_interest_vertices = [
        (0, height),
        (width / 2 - 80, height / 1.4),
        (width / 2 + 80, height / 1.4),
        (width, height)
    ]

    try:
        img_data = camera_instance.getImage()
        img_data = np.frombuffer(img_data, dtype=np.uint8).reshape(height, width, 4)
        if type(img_data) is np.ndarray:
            lines_hp = image_processing(img_data)
            if lines_hp is not None and len(lines_hp):
                right_left_finder(lines_hp)
                center_of_lane_res = center_of_lane(m_lane_change, m_gps)
                if flag:
                    draw_and_display_lines(display_front, img_data, reduced_lines)
                else:
                    Log.append("no-display-screen")

        if center_of_lane_res is None:
            error_Log.append("[LANE MANAGEMENT] error center could't found...")
    except Exception as e:
        error_Log.append("ERROR IN LANE MANAGEMENT:%s" % e)
    Log.append(str(time.time() - start_time))
    with open("Logs\Lane_Log.csv", 'a') as file:
        wr = writer(file, quoting=QUOTE_ALL)
        wr.writerow(Log)
    if len(error_Log):
        with open("Logs\error_Log.csv", 'a', newline="") as file:
            wr = writer(file, quoting=QUOTE_ALL)
            wr.writerow(error_Log)
    return center_of_lane_res


if __name__ == '__main__':
    main()
