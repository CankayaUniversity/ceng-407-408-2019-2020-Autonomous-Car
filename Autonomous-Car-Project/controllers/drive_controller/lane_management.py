"""

    This script responsible of image processing for detecting lines, checking if neighbour lane exists.

    Büşra Nur Bahadır 201511006

                                                                                                            """

import cv2
import numpy as np
from controller import Display

height = 256
width = 512
region_of_interest_vertices = [
    (70, height),
    (width / 2 - 80, height / 1.6),
    (width / 2 + 80, height / 1.6),
    (width - 70, height),
]
right_lines = np.empty((0, 4), dtype=np.uint8)
left_lines = np.empty((0, 4), dtype=np.uint8)
has_right_lane = False
has_left_lane = False
img_ref = None

"""------------------------------------------------------------------------------------------------------------"""


# For masking other parts of the image than our interest
def region_of_interest(image, vertices):
    # blank matrix with same h,w with the image
    mask = np.zeros((height, width), dtype=np.uint8)
    # mask everything other than region of interest
    cv2.fillPoly(mask, vertices, 255)
    # return the image
    res = cv2.bitwise_and(image, image, mask=mask)
    return res


"""------------------------------------------------------------------------------------------------------------"""


# Reducing lines
def reduce_lines(line_arr):
    cnt = 0
    size = len(line_arr)
    while cnt < size:
        lst = []
        line1 = line_arr[cnt]
        for cnt2 in range(cnt + 1, size, 1):
            line2 = line_arr[cnt2]
            t = abs(line1 - line2)
            if np.min(t) < 40:
                lst.append(cnt2)
        line_arr = np.delete(line_arr, lst, axis=0)
        size = size - len(lst)
        cnt += 1
    return line_arr


"""------------------------------------------------------------------------------------------------------------"""


# Deciding which line belongs to which direction
def right_left_finder(lines):
    global left_lines
    global right_lines
    global has_right_lane
    global has_left_lane
    reduced_lines = np.empty((0, 4), dtype=np.uint8)
    for n in range(0, len(lines)):
        line = lines[n][0]
        # Calculate slope
        if line[2] - line[0] == 0.:  # avoiding division by 0
            # infinite slope
            break
        elif line[1] - line[3] == 0.:
            # horizontal line
            break
        else:
            slope = (line[3] - line[1]) / (line[2] - line[0])
            if 0.2 < abs(slope) < 0.5:
                if slope > 0 and line[0] > (width / 2) + 10 and line[2] > (width / 2) + 10:
                    right_lines = np.append(right_lines, np.array([line]), axis=0)
                elif slope < 0 and line[0] < (width / 2) - 10 and line[2] < (width / 2) - 10:
                    left_lines = np.append(left_lines, np.array([line]), axis=0)
    if len(right_lines) > 1:
        right_lines = reduce_lines(right_lines)
        reduced_lines = np.append(reduced_lines, np.array(right_lines), axis=0)
    if len(left_lines) > 1:
        left_lines = reduce_lines(left_lines)
        reduced_lines = np.append(reduced_lines, np.array(left_lines), axis=0)

    if len(right_lines) > 1:
        has_right_lane = True
    else:
        has_right_lane = False
    if len(left_lines) > 1:
        has_left_lane = True
    else:
        has_left_lane = False
    return reduced_lines


center_of_the_lane = None

"""------------------------------------------------------------------------------------------------------------"""


#  to find result as a gps value
#  (target on display)*gps/( gps's value on display)
def toGPS_val(val):
    global gps
    res = (val * gps) / (width / 2)
    return res


"""------------------------------------------------------------------------------------------------------------"""


# Find center of lanes and Draw lines on the original image
def draw_lines(image, lines):
    E = 75  # a constant value to stay this much far from lane (only used when we can detect one line)
    global left_lines
    global right_lines
    global center_of_the_lane
    global lane_change

    """------------------------------------------------------------------------------------------------------------"""
    # switch lane directions to change lane
    if lane_change == "right" and len(right_lines) > 0:
        left_lines = right_lines
        right_lines = np.empty((0, 4), dtype=np.uint8)
    elif lane_change == "left" and len(left_lines) > 0:
        right_lines = left_lines
        left_lines = np.empty((0, 4), dtype=np.uint8)
    """------------------------------------------------------------------------------------------------------------"""
    # decide center of line
    if len(right_lines) > 0:
        r_mean = np.mean(right_lines, axis=0)
    if len(left_lines) > 0:
        l_mean = np.mean(left_lines, axis=0)
    try:
        if len(right_lines) == 0 and len(left_lines) > 0:
            center_of_the_lane = toGPS_val(l_mean[0] + E + 50)
        elif len(left_lines) == 0 and len(right_lines) > 0:
            center_of_the_lane = toGPS_val(r_mean[0] - E)
        elif len(right_lines) > 0 and len(left_lines) > 0:
            center_of_the_lane = toGPS_val(((((((r_mean[2] + r_mean[0]) / 2) - ((l_mean[2] + l_mean[0]) / 2)) / 2) + (
                    (l_mean[2] + l_mean[0]) / 2))))
    except ValueError:
        print("An exception occurred")
        center_of_the_lane = gps
    """------------------------------------------------------------------------------------------------------------"""
    # draw center of line to display screen
    blank_img = np.zeros((image.shape[0], image.shape[1], image.shape[2]), dtype=np.uint8)
    """------------------------------------------------------------------------------------------------------------"""
    for x in range(0, len(lines)):
        line = lines[x]
        # I don't know why but it does not works when I write values directly as line[0]
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        pts = np.array([[x1, y1], [x2, y2]], np.int32)
        cv2.polylines(blank_img, [pts], True, (0, 255, 0), thickness=3)
    """------------------------------------------------------------------------------------------------------------"""
    # Merge two images
    image = cv2.addWeighted(image, 0.8, blank_img, 1, 0.0)
    # cv2.imshow("center",image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return image


"""------------------------------------------------------------------------------------------------------------"""


# Main part of image processing for lane detection
def lane_detecting(image_data):
    global right_lines
    global left_lines

    # CV2
    image_np = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 4)
    # print(type(np_arr))
    # print(np_arr.shape)
    gray_img = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
    # if image is all black no need for processing
    if cv2.countNonZero(gray_img) == 0:
        print("couldn't take image data")
        return None

    # Convert to HSV
    hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
    result = region_of_interest(hsv, np.array([region_of_interest_vertices], np.int32))

    # set color limits
    low_val = (0, 0, 0)
    high_val = (179, 45, 96)

    # threshold
    mask = cv2.inRange(hsv, low_val, high_val)
    # remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((8, 8), dtype=np.uint8))
    # close mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((20, 20), dtype=np.uint8))

    # improve by drawing the convexhull
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        hull = cv2.convexHull(cnt)
        cv2.drawContours(mask, [hull], 0, 255, -1)

    # erode
    mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel=np.ones((5, 5), dtype=np.uint8))
    road_hsv = cv2.bitwise_and(hsv, hsv, mask=mask)

    # set new color limits
    low_val = (0, 0, 102)
    high_val = (179, 255, 255)

    # new threshold
    mask2 = cv2.inRange(road_hsv, low_val, high_val)
    masked = cv2.bitwise_and(image_np, image_np, mask=mask2)
    result = region_of_interest(masked, np.array([region_of_interest_vertices], np.int32))
    result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    rho = 1  # distance resolution in pixels
    theta = np.pi / 180  # angular resolution in radians
    threshold = 5  # minimum number of pixels
    min_line_length = 10  # minimum distance between lines
    max_line_gap = 15  # maximum gap between line segments
    lines = cv2.HoughLinesP(result, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

    if lines is not None:
        reduced_lines = right_left_finder(lines)
        image_with_lines = draw_lines(image_np, reduced_lines)
        right_lines = np.empty([0, 4], dtype=np.uint8)  # clear
        left_lines = np.empty([0, 4], dtype=np.uint8)  # clear
        return image_with_lines
    else:
        print("lanes can not found with image processing")
        # cv2.imshow("Result", result)
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Image", image_np)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return None


"""------------------------------------------------------------------------------------------------------------"""


def display_front_cam():
    """ To show merged image on display screen as a video """
    global img_ref
    if img_ref is not None:
        img_ref = display_front.imageDelete(img_ref)
    img_data = front_camera.getImage()

    if type(img_data) is bytes:
        img_n_arr = lane_detecting(img_data)
        if img_n_arr is not None:
            # convert numpy array for display
            try:
                img = np.array(np.swapaxes(img_n_arr, 0, 1)).tolist()
                # print(type(img))
                # print(img)
                img_ref = display_front.imageNew(img, Display.RGBA, height=512, width=256)

                # blend for opacity
                display_front.imagePaste(img_ref, 0, 0, blend=False)
            except ValueError:
                print("value error on display screen")


"""------------------------------------------------------------------------------------------------------------"""


# main to call other functions
def main(m_display_front, m_front_camera, m_auto_drive, m_gps, m_lane_change):
    global display_front, front_camera
    global auto_drive, gps
    global lane_change
    lane_change = m_lane_change
    gps = m_gps
    front_camera = m_front_camera
    display_front = m_display_front
    auto_drive = m_auto_drive
    display_front_cam()
    if center_of_the_lane is not None:
        return round(center_of_the_lane)
    else:
        print("center could't found")
        return gps


if __name__ == '__main__':
    main()
