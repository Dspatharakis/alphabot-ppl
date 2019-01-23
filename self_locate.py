import os
import subprocess
import RPi.GPIO as GPIO
from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
from dna import *
import not_found_error

Dna = Dna()
S1 = 27 # or 22

class SelfLocator(): 
    
    def __init__(self, step):
        self.camera = PiCamera()
        self.camera.resolution = (2592, 1944)
        self.step = step # 1Deg ~= 11.11ms

    def dna_from_beacons(self):
        beacons_found = 0
        pulse_width = 2650
        distance = []
        angle = []
        color = []

        # Distance and angle from at least 2 Beacons is needed for accurate localisation
        while ((beacons_found < 2) and self.step > 100):
            while (pulse_width >= 900):
                # Scan area with a 30-angle step
                pulse_width -= self.step
                with open(os.devnull, 'wb') as devnull:
                    subprocess.check_call(['sudo', 'python', 'turn_head.py', '-s', '27', '-w', str(pulse_width)], stdout=devnull, stderr=subprocess.STDOUT)
                sleep(1)
                candidate = open('candidate'+str(pulse_width)+'.jpg', 'wb')
                self.camera.capture(candidate)
                candidate.close()
                try:
                    d, a, c = Dna.find_distance_and_angle('candidate'+str(pulse_width)+'.jpg')
                    # Check for already-found color
                    if c not in color:
                        distance.append(round(d, 2))
                        angle.append(round((pulse_width-1650)/11.11 - a, 2))
                        color.append(c)
                        beacons_found += 1
                        print("Beacons found: " + str(beacons_found))
                    else:
                        print("Beacon already found, not updating")
                    print("-------------------------------\n")
                except NotFoundError:
                    continue
                if (beacons_found == 2):
                    break

            # If no 2 beacons are found, try again with a smaller step
            self.step = self.step / 2 

        print("-------------------------------")
        print distance, angle, color

        # Straigthen camera's position before exiting
        with open(os.devnull, 'wb') as devnull:
            subprocess.check_call(['sudo', 'python', 'turn_head.py', '-s', '27', '-w', '1600'], stdout=devnull, stderr=subprocess.STDOUT)
        
        return distance, angle, color
