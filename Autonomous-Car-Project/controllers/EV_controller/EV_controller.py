"""
    EV_driver controller.
 This drive controller made for testing emergency vehicles behaviours.
 This script responsible for controlling siren,emergency signal,leds and speed of the vehicle.
 with the keyboard input e and q you can start or stop siren.

 Büşra Nur Bahadır 201511006

"""
import struct
import os
import time

import keyboard
from vehicle import Driver

print("with the keyboard input e and q you can start or stop siren.")


def main():
    # Emergency mode
    mode = True
    while driver.step() != -1:

        if keyboard.is_pressed('e'):
            mode = True
            # print("key e pressed ")
        elif keyboard.is_pressed('q'):
            mode = False
            # print("key q pressed ")

        if not file:
            print("sound file can not found")
        elif file and mode:
            speaker.playSound(left=speaker, right=speaker, sound=path, volume=0.7, pitch=1.0,
                              balance=0.0, loop=True)
        elif file and not mode:
            speaker.stop(sound=path)

        # If siren is playing then sends emergency signal to cars in front of it in 4 metre.
        if speaker.isSoundPlaying(path) or mode:
            emergency_message = struct.pack("?", mode)
            emitter.send(emergency_message)

        if not red.get():
            red.set(1)
            blue.set(0)
        else:
            red.set(0)
            blue.set(1)


if __name__ == '__main__':
    # get driver instance
    driver = Driver()
    # set speed of the vehicle
    driver.setCruisingSpeed(100)
    speaker = driver.getSpeaker("Siren")

    # led sensor
    red = driver.getLED("red")
    red.set(0)
    blue = driver.getLED("blue")
    blue.set(0)

    # Emitter sensor to provide communication
    emitter = driver.getEmitter("emitter")
    path = "sounds/AmbulanceSiren.wav"
    path = os.path.abspath(path)
    file = os.path.isfile(path)
    main()
