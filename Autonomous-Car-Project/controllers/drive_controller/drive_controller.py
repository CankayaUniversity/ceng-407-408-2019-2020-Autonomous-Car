"""
    Drive controller.

    This script responsible of managing all the modules of autonomous car project.

    Büşra Nur Bahadır 201511006

                                                                                 """

import math
import struct
import sys
import DataFusion

from controller import Node
from vehicle import Driver
import Obj_Recognition


def main():
    emergency_message, prev_gps, gps_val = None, None, None
    # Main loop
    while auto_drive.step() != -1:
        for lidar in lidars:
            lidar.getPointCloud()
        # if any emergency message received by the car
        """ If an Emergency Vehicle in the emergency state closer than 4 metre it sends emergency message
        to cars in front of it and other cars has sends messages as a chain to clear the way """
        if receiver.getQueueLength() > 0:
            message = receiver.getData()
            # #  for sending emergency message to AutoCars front of our AutoCar
            # emitter.send(message)
            emergency_message = struct.unpack("?", message)
            emergency_message = emergency_message[0]
            receiver.nextPacket()
        else:
            emergency_message = False
        gps_val = round(math.sqrt(gps.getValues()[0] ** 2 + gps.getValues()[2] ** 2), 0)
        if gps_val is None or math.isnan(gps_val):
            print("couldn't get gps value")
        else:
            prev_gps = gps_val
        if prev_gps is not None and gps_val is None:
            gps_val = prev_gps
        if gps_val is not None:
            obj_data, LIDAR_data = Obj_Recognition.main(lidars, dist_sensors, front_camera, back_camera)
            DataFusion.main(auto_drive, gps_val, obj_data, LIDAR_data, emergency_message, display_front,
                            front_camera, dist_sensors)


"""------------------------------------------------------------------------------------------------------------"""

if __name__ == '__main__':
    # empty files
    open("OBJ", 'w').close()
    open("PID", 'w').close()
    open("Lane", 'w').close()
    # define auto_driver instance
    auto_drive = Driver()
    auto_drive.setAntifogLights(True)
    auto_drive.setDippedBeams(True)
    TIME_STEP = int(auto_drive.getBasicTimeStep())

    """ Defining and enabling sensors as lidar,radar, gps"""
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

    dist_sensors = {}
    for name in dist_sensor_names:
        dist_sensors[name] = auto_drive.getDistanceSensor("distance sensor " + name)
        dist_sensors[name].enable(TIME_STEP*2)

    # GPS
    gps = auto_drive.getGPS("gps")
    gps.enable(TIME_STEP)

    # Compass
    compass = auto_drive.getCompass("compass")
    compass.enable(TIME_STEP)

    # get and enable front camera
    front_camera = auto_drive.getCamera("camera1")
    front_camera.enable(TIME_STEP)
    front_camera.recognitionEnable(TIME_STEP)

    # get and enable back camera
    back_camera = auto_drive.getCamera("camera2")
    back_camera.enable(TIME_STEP*2)
    back_camera.recognitionEnable(TIME_STEP*5)

    # Get the display devices.
    # The display can be used to visually show the tracked position.
    # For showing lane detection
    display_front = auto_drive.getDisplay('display')
    display_front.setColor(0xFF00FF)

    # to establish communication between Emergency Vehicle
    receiver = auto_drive.getReceiver("receiver")
    receiver.enable(TIME_STEP*5)

    # to establish communication between other vehicles
    emitter = auto_drive.getEmitter("emitter")

    # lidars
    lidars = []

    for i in range(auto_drive.getNumberOfDevices()):
        device = auto_drive.getDeviceByIndex(i)
        if device.getNodeType() == Node.LIDAR:
            lidars.append(device)
            device.enable(TIME_STEP*10)
            device.enablePointCloud()
    if not lidars:
        sys.exit("This vehicle has no 'Lidar' node.")

    """------------------------------------------------------------------------------------------------------------"""
    # set first values for PID controller
    auto_drive.setCruisingSpeed(60)
    auto_drive.setSteeringAngle(0)
    """------------------------------------------------------------------------------------------------------------"""
    main()
