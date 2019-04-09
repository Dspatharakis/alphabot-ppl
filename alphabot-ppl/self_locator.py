import os
import subprocess
import RPi.GPIO as GPIO
from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
from dna import *
import alphabot_exceptions
import yaml
import requests 
import json 
import time 

CONFIG =yaml.load(open("../config.yaml"))
S1 = str(CONFIG["camera"]["vertical_servo_pin"])
S2 = str(CONFIG["camera"]["horizontal_servo_pin"])
CAMERA_RESOLUTION = CONFIG["camera"]["resolution"]
SCANNING_STEP = CONFIG["camera"]["scanning_step"]
#OFFLOAD = CONFIG["offload"]
POST_URL = CONFIG["post_url"]

class SelfLocator(): 
    
    def __init__(self, step):
        self.camera = PiCamera()
        self.camera.resolution = CAMERA_RESOLUTION
        self.step = SCANNING_STEP # 1Deg ~= 11.11ms

    # Scan the area (at most twice) seeking for Beacons. Capture pictures and calculate distance and angle 
    # from them.
    def dna_from_beacons(self,OFFLOAD):
        beacons_found = 0
        step = self.step
        if OFFLOAD:
            print("\n---------- Image Processing is offloaded to the Edge Server ----------\n")
        else:
            print("\n---------- Image Processing is performed locally ----------\n")
        # Bringing the camera to an upright position
        print("Bringing camera to the upright position...\n")
        with open(os.devnull, 'wb') as devnull:
            subprocess.check_call(['sudo', 'python', 'turn_head.py', '-s', S2, '-w', 
                "1600"], stdout=devnull, stderr=subprocess.STDOUT)
        # Distance and angle from at least 2 Beacons is needed for accurate localisation
        while (step > 200):
            print("Scanning Step: " + str(step))
            beacons_found = 0
            pulse_width = 2650
            distance = []
            angle = []
            color = []
            while (pulse_width >= 900):
                # Scan area with a 30-angle step
                pulse_width -= step
                with open(os.devnull, 'wb') as devnull:
                    subprocess.check_call(['sudo', 'python', 'turn_head.py', '-s', S1, '-w', 
                        str(pulse_width)], stdout=devnull, stderr=subprocess.STDOUT)
                sleep(1)
                candidate = open('images/candidate' + str(pulse_width) + '.jpg', 'wb')
                self.camera.capture(candidate)
                candidate.close()
                try:
                    start_time = time.time()
                    # Using either the edge server to perform the image processing or do it locally
                    if OFFLOAD:
                        payload = { "time" : time.time()}
                        files = [("time", ("datas", json.dumps(payload), "application/json")), 
                                ("file", ("temp", open('./images/candidate' + str(pulse_width) + ".jpg", "rb"),'application/octet-stream'))]
                        r = requests.post(POST_URL, files=files)
                        end_time = time.time()-start_time
                        print "Total time for Image recognition: " +str(end_time)

			if (r.status_code == 404): 
                            raise BeaconNotFoundError
                        d, a, c, z0 = json.loads(r.text)
			print "CPU availability of server: " + str(z0)
                    else :
                        d, a, c = Dna().find_distance_and_angle('images/candidate'+str(pulse_width)+'.jpg')
                        end_time = time.time()-start_time
                        #print "Total time for Image recognition: " +str(end_time)

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
                except BeaconNotFoundError:
                    end_time = time.time()
                    end_time = end_time - start_time
                    #print "Total time for Image recognition: " +str(end_time)
                    continue
                if (beacons_found == 2):
                    print("-------------------------------")
                    # Allign camera's position before exiting (the camera's weight messes with movement)
                    with open(os.devnull, 'wb') as devnull:
                        subprocess.check_call(['sudo', 'python', 'turn_head.py', '-s', S1, '-w', '1600'], 
                                stdout=devnull, stderr=subprocess.STDOUT)
                    return distance, angle, color, z0 

            # If no 2 beacons are found, try again with a smaller step
            step = step / 2 
        
        print("-------------------------------")
        # Allign camera's position before exiting (the camera's weight messes with movement)
        with open(os.devnull, 'wb') as devnull:
            subprocess.check_call(['sudo', 'python', 'turn_head.py', '-s', S1, '-w', '1600'], 
                    stdout=devnull, stderr=subprocess.STDOUT)

        raise InsufficientLocalizationInfoError
        quit() 
        
