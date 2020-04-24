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
    (0, height),
    (width / 2 - 100, height / 1.6),
    (width / 2 + 100, height / 1.6),
    (width, height),
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
        if line[2] - line[0] == 0.:  # corner case, avoiding division by 0
            # practically infinite slope slope = 999.
            break
        elif line[1] - line[3] == 0.:
            # horizontal line  slope = 0.
            break
        else:
            slope = (line[3] - line[1]) / (line[2] - line[0])
            if 0.2 < abs(slope) < 1.:
                if slope > 0 and line[0] > (width / 2)+10 and line[2] > (width / 2)+10:
                    right_lines = np.append(right_lines, np.array([line]), axis=0)
                elif slope < 0 and line[0] < (width / 2)-10 and line[2] < (width / 2)-10:
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
    res = (val * gps) / (width/2)
    return res


"""------------------------------------------------------------------------------------------------------------"""


# Find center of lanes and Draw lines on the original image
def draw_lines(image, lines):
    E = width/3  # a constant value to stay this much far from lane (only used when we can detect one line)
    global left_lines
    global right_lines
    global center_of_the_lane
    if len(right_lines) > 0:
        r_mean = np.mean(right_lines, axis=0)
    if len(left_lines) > 0:
        l_mean = np.mean(left_lines, axis=0)
    try:
        if len(right_lines) == 0 and len(left_lines) > 0:
            center_of_the_lane = toGPS_val(l_mean[0]+E)
        elif len(left_lines) == 0 and len(right_lines) > 0:
            center_of_the_lane = toGPS_val(r_mean[0]-E)
        elif len(right_lines) > 0 and len(left_lines) > 0:
            center_of_the_lane = toGPS_val(((((((r_mean[2] + r_mean[0]) / 2) - ((l_mean[2] + l_mean[0]) / 2)) / 2) + (
                    (l_mean[2] + l_mean[0]) / 2))))
    except ValueError:
        print("An exception occurred")
        center_of_the_lane = gps
    blank_img = np.zeros((image.shape[0], image.shape[1], image.shape[2]), dtype=np.uint8)

    for x in range(0, len(lines)):
        line = lines[x]
        # I don't know why but it does not works when I write values directly as line[0]
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        pts = np.array([[x1, y1], [x2, y2]], np.int32)
        cv2.polylines(blank_img, [pts], True, (0, 255, 0), thickness=3)

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
    np_arr = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 4)
    # print(type(np_arr))
    # print(np_arr.shape)
    # convert image to gray image
    gray_img = cv2.cvtColor(np_arr, cv2.COLOR_BGR2GRAY)
    # if image is all black no need for processing
    if cv2.countNonZero(gray_img) == 0:
        print("couldn't take image data")
        return None

    # Convert to HLS color space
    hls = cv2.cvtColor(np_arr, cv2.COLOR_BGR2HLS)

    # Apply bilateral filter with d = 15,
    # sigmaColor = sigmaSpace = 75.
    hls = cv2.bilateralFilter(hls, 15, 75, 75)

    # Crop image
    cropped_image = region_of_interest(hls, np.array([region_of_interest_vertices], np.int32))
    # cv2.imshow("cropped", cropped_image)
    # cv2.waitKey(0)

    # White areas HLS value
    white_lower = np.array([np.round(0 / 2), np.round(0.75 * 255), np.round(0.00 * 255)])
    white_upper = np.array([np.round(360 / 2), np.round(1.00 * 255), np.round(0.30 * 255)])
    white_mask = cv2.inRange(cropped_image, white_lower, white_upper)

    # Yellow areas HLS value
    yellow_lower = np.array([np.round(40 / 2), np.round(0.00 * 255), np.round(0.35 * 255)])
    yellow_upper = np.array([np.round(80 / 2), np.round(1.00 * 255), np.round(1.00 * 255)])
    yellow_mask = cv2.inRange(cropped_image, yellow_lower, yellow_upper)

    # Calculate combined mask, and masked image
    mask = cv2.bitwise_or(yellow_mask, white_mask)
    masked = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)

    grey_image = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("gray", grey_image)
    # cv2.waitKey(0)
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(grey_image, (kernel_size, kernel_size), 0)
    # cv2.imshow("blur_gray", blur_gray)
    # cv2.waitKey(0)
    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
    # cv2.imshow("edges", edges)
    # cv2.waitKey(0)
    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 10  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 20  # minimum number of pixels making up a line
    max_line_gap = 30  # maximum gap in pixels between connect-able line segments
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)

    if lines is not None:
        reduced_lines = right_left_finder(lines)
        image_with_lines = draw_lines(np_arr, reduced_lines)
        right_lines = np.empty([0, 4], dtype=np.uint8)
        left_lines = np.empty([0, 4], dtype=np.uint8)
        return image_with_lines
    else:
        print("lanes can not found with image processing")
        return gps


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


def main(m_display_front, m_front_camera, m_auto_drive, m_gps):
    global display_front, front_camera
    global auto_drive, gps
    gps = m_gps
    front_camera = m_front_camera
    display_front = m_display_front
    auto_drive = m_auto_drive
    display_front_cam()
    if center_of_the_lane is not None:
        return round(center_of_the_lane, 1)
    else:
        print("center could't found")
        return None


if __name__ == '__main__':
    main()
