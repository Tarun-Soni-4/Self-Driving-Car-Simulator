import struct
import os
import time
import keyboard
from vehicle import Driver
print("with the keyboard input e and q you can start or stop siren.")
def main():
    mode = True
    while driver.step() != -1:
        if keyboard.is_pressed('e'):
            mode = True
        elif keyboard.is_pressed('q'):
            mode = False
        if not file:
            print("sound file can not found")
        elif file and mode:
            speaker.playSound(left=speaker, right=speaker, sound=path, volume=0.7, pitch=1.0, balance=0.0, loop=True)
        elif file and not mode:
            speaker.stop(sound=path)
        if speaker.isSoundPlaying(path) or mode:
            emergency_message = struct.pack("?", mode)
            emitter.send(emergency_message)
        if not red.get():
            red.set(1)
            blue.set(0)
        else:
            red.set(0)
            blue.set(1)
driver = Driver()
driver.setCruisingSpeed(100)
speaker = driver.getSpeaker("Siren")
red = driver.getLED("red")
red.set(0)
blue = driver.getLED("blue")
blue.set(0)
emitter = driver.getEmitter("emitter")
path = "sounds/AmbulanceSiren.wav"
path = os.path.abspath(path)
file = os.path.isfile(path)
