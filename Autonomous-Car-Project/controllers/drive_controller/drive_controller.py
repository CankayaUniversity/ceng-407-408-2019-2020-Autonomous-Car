"""
    Drive controller.

    This script responsible of managing all the modules of autonomous car project.

    Büşra Nur Bahadır 201511006

                                                                                 """
import multiprocessing as mp
import os
import struct
import subprocess
import sys
import time
from math import sqrt, atan2, pi
import psutil

import DataFusion
import Obj_Recognition
import voice_assistant as va
from controller import Node
from vehicle import Driver
from csv import writer, QUOTE_ALL


def VA_call(m_order_queue, shut_event, respond_dict):
    """ Runs voice assistant
           Defining and enabling sensors.
           :param shut_event: to break while loop when simulator stops
           :type shut_event: Event
           :param respond_dict: dictionary to send value VA process
           :param m_order_queue:
           :type  m_order_queue: multiprocessing.Queue To provide communication between voice assistant process.
           :rtype None

    """
    Log = dict()
    error_Log = list()
    va.VA_speaks("hi I'm Jarvis")
    while not shut_event.is_set():
        try:
            order, Log, error_Log = va.main(Log, error_Log)
            if order == 5:
                break
            if order is not None:
                m_order_queue.put(order)
            if bool(respond_dict):
                va.VA_response_fun(respond_dict)
        except RuntimeError as e:
            error_Log.append("[VA] IN VA_CALL: %s" % e)
        if bool(Log):
            with open("Logs\VA_Log.csv", 'a') as file:
                wr = writer(file, quoting=QUOTE_ALL)
                for key, value in Log.items():
                    wr.writerow([key, value])
        if len(error_Log):
            with open("Logs\error_Log.csv", 'a', newline="") as file:
                wr = writer(file, quoting=QUOTE_ALL)
                wr.writerow(error_Log)


def auto_drive_call(m_order_queue, respond_dict):
    """
        Runs drive instance in the simulation.
            Defining and enabling sensors.
            :param respond_dict: dictionary to send value VA process
            :param m_order_queue:
            :type  m_order_queue: multiprocessing.Queue To provide communication between voice assistant process.
            :rtype None

    """
    # Driver initialize
    auto_drive = Driver()
    auto_drive.setAntifogLights(True)
    auto_drive.setDippedBeams(True)
    TIME_STEP = int(auto_drive.getBasicTimeStep())

    # distance sensors
    dist_sensor_names = [
        "front",
        "front right 0",
        "front right 1",
        "front right 2",
        "front right 3",
        "front left 0",
        "front left 1",
        "front left 2",
        "front left 3",
        "rear",
        "rear left",
        "rear right",
        "right",
        "left"]

    dist_sensors = {}
    for name in dist_sensor_names:
        dist_sensors[name] = auto_drive.getDistanceSensor("distance sensor " + name)
        dist_sensors[name].enable(TIME_STEP)

    # GPS
    gps = auto_drive.getGPS("gps")
    gps.enable(TIME_STEP)

    # Compass
    compass = auto_drive.getCompass("compass")
    compass.enable(TIME_STEP)

    # get and enable front camera
    front_camera1 = auto_drive.getCamera("front camera 1")
    front_camera1.enable(TIME_STEP)

    front_camera2 = auto_drive.getCamera("front camera 2")
    front_camera2.enable(TIME_STEP)
    front_camera3 = auto_drive.getCamera("front camera 3")
    front_camera3.enable(TIME_STEP)

    front_cams = {
        "right": front_camera2,
        "left": front_camera1,
        "center": front_camera3
    }

    # get and enable back camera
    back_camera = auto_drive.getCamera("camera2")
    back_camera.enable(TIME_STEP * 10)
    # back_camera.recognitionEnable(TIME_STEP * 10)

    # Get the display devices.
    # The display can be used to visually show the tracked position.
    # For showing lane detection
    display_front = auto_drive.getDisplay('display')
    display_front.setColor(0xFF00FF)
    # To establish communication between Emergency Vehicle
    receiver = auto_drive.getReceiver("receiver")
    receiver.enable(TIME_STEP)

    # To establish communication between other vehicles
    emitter = auto_drive.getEmitter("emitter")

    # lidar devices
    lidars = []

    Log = list()
    error_Log = list()

    for i in range(auto_drive.getNumberOfDevices()):
        device = auto_drive.getDeviceByIndex(i)
        if device.getNodeType() == Node.LIDAR:
            lidars.append(device)
            device.enable(TIME_STEP * 10)
            device.enablePointCloud()
    if not lidars:
        error_Log.append(" [ DRIVER CALL] This vehicle has no 'Lidar' node.")

    # Set first values
    auto_drive.setCruisingSpeed(40)
    auto_drive.setSteeringAngle(0)
    VA_order, emergency_message, prev_gps, gps_val = None, None, None, None

    # Main Loop
    while auto_drive.step() != -1:
        start_time = time.time()
        # for lidar in lidars:
        #     lidar.getPointCloud()
        if m_order_queue.qsize() > 0:
            VA_order = m_order_queue.get()
        else:
            VA_order = None
        """ If an Emergency Vehicle in the emergency state closer than 4 metre it sends emergency message
             to cars in front of it and other cars has sends messages as a chain to clear the way """
        if receiver.getQueueLength() > 0:
            message = receiver.getData()
            #  for sending emergency message to AutoCars front of our AutoCar
            # emitter.send(message)
            emergency_message = struct.unpack("?", message)
            emergency_message = emergency_message[0]
            receiver.nextPacket()
        else:
            emergency_message = False

        gps_val = round(sqrt(gps.getValues()[0] ** 2 + gps.getValues()[2] ** 2), 2)
        if gps_val is None:
            error_Log.append("[DRIVER CALL] couldn't get gps value..")
        else:
            prev_gps = gps_val

        if prev_gps is not None and gps_val is not None:
            gps_val = prev_gps
            if gps_val is not None:
                # To calculate direction of the car
                cmp_val = compass.getValues()
                angle = ((atan2(cmp_val[0], cmp_val[2])) * (180 / pi)) + 180
                # goes on Z axis
                if 335 <= angle <= 360 or 0 <= angle <= 45 or 135 <= angle <= 225:
                    axis = 1
                # goes on X axis
                elif 225 <= angle < 335 or 45 <= angle < 135:
                    axis = 0
                obj_data, LIDAR_data = Obj_Recognition.main(dist_sensor_names, lidars, dist_sensors,
                                                            front_cams, back_camera)
                DataFusion.main(auto_drive, gps_val, obj_data, LIDAR_data, emergency_message, display_front,
                                front_cams, dist_sensors, VA_order, respond_dict, gps, axis)
            else:
                error_Log.append("[DRIVER CALL] couldn't get gps value..")
        Log.append(str(time.time() - start_time))
        with open("Logs\Driver_Log.csv", 'a') as file:
            wr = writer(file, quoting=QUOTE_ALL)
            wr.writerow(Log)
        if len(error_Log):
            with open("Logs\error_Log.csv", 'a', newline="") as file:
                wr = writer(file, quoting=QUOTE_ALL)
                wr.writerow(error_Log)


"""------------------------------------------------------------------------------------------------------------"""


def server_check(va_pid, drive_pid, shut_event):
    shut_man_pid = os.getpid()
    while not shut_event.is_set():
        if not psutil.pid_exists(drive_pid):
            shut_event.set()
            subprocess.check_output("Taskkill /PID %d /F" % va_pid)
            subprocess.check_output("Taskkill /PID %d /F" % shut_man_pid)
        elif not psutil.pid_exists(va_pid):
            subprocess.check_output("Taskkill /PID %d /F" % drive_pid)
            subprocess.check_output("Taskkill /PID %d /F" % shut_man_pid)


def Log_handler():
    open("Data\OBJ", 'w').close()
    open("Data\PID", 'w').close()
    open("Data\Lane", 'w').close()
    with open("Logs\PID_Log.csv", 'w', newline='') as file:
        f_writer = writer(file)
        f_writer.writerow(["speed", "speed_cal_t", "position", "target_position", "angle", "angle_cal_t", "PID_cal_t"])
    with open("Logs\Lane_Log.csv", 'w', newline='') as file:
        f_writer = writer(file)
        f_writer.writerow(["imgPr_cal_t", "reduce_right_lines_cal_t", "reduce_left_lines_cal_t",
                           "left_right_cal_t", "cl_cal_t", "display_t", "Lane_cl_t"])
    with open("Logs\error_Log.csv", 'w', newline='') as file:
        f_writer = writer(file)
        f_writer.writerow(["ERROR"])

    with open("Logs\DF_Log.csv", 'w', newline='') as file:
        f_writer = writer(file)
        f_writer.writerow(["DF_cal_t"])

    with open("Logs\OBJ_Log.csv", 'w', newline='') as file:
        f_writer = writer(file)
        f_writer.writerow(["OBJ_cal_t"])

    with open("Logs\VA_Log.csv", 'w', newline='') as file:
        f_writer = writer(file)
        f_writer.writerow(["request", "VA respond"])

    with open("Logs\Driver_Log.csv", 'w', newline='') as file:
        f_writer = writer(file)
        f_writer.writerow(["Total Time Cost"])


def main():
    """ Main function to start multiprocessing.
        ----------
        OBJ : To store and load Object Recognition data
        PID : To store and load PID data
        Lane : To store and load Lane Management data
        ----------
        p1 : process 1 to run driver class of simulation
        p2: process 2 to run voice assistant
        order_queue : Multiprocessing queue for receiving orders from voice assistant
        shut_pid: to kill voice assistant process when drive process is no longer exist.

        """
    # empty files
    Log_handler()
    manager = mp.Manager()
    respond_dict = manager.dict()
    respond_dict.clear()
    order_queue = mp.Queue()
    shut_event = mp.Event()
    p1 = mp.Process(target=auto_drive_call, args=(order_queue, respond_dict,))
    p2 = mp.Process(target=VA_call, args=(order_queue, shut_event, respond_dict,))
    p1.start()
    p2.start()
    shut_pid, va_pid = p1.pid, p2.pid
    p3 = mp.Process(target=server_check, args=(va_pid, shut_pid, shut_event))
    p3.start()
    p1.join()
    p2.join()
    p3.join()
    print("Is server process alive:", p3.is_alive())
    print("Is voice assistant process alive:", p2.is_alive())
    print("Is driver process alive:", p1.is_alive())
    if not (p3.is_alive() or p2.is_alive() or p1.is_alive()):
        print("all processes terminated")
    sys.exit()


if __name__ == '__main__':
    main()
