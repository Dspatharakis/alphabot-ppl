#!/usr/bin/env python
import os
import requests
import json
import time
import sys
import subprocess
import csv
import json
import numpy as np

def main():
    start_time = time.time()
    post_url = "http://0.0.0.0:8000/cpu"
    mean_cpu = 0
    counter = 0
    s = 0.75
    container_name = subprocess.check_output(["docker", "ps", "-aqf", "name=edge"])
    while True:
        measured_cpu = subprocess.check_output(["docker", "stats", "--no-stream", "--format", "{{ .CPUPerc }}"] )# , ">", "./log.txt"])
        measured_cpu = (float(measured_cpu[:-2]))
        if measured_cpu > 0.1 :
            mean_cpu += measured_cpu
            counter += 1 
         
        now_time = time.time ()
        #print now_time - start_time 
        if now_time-start_time >  10 : 
            start_time = time.time()
            if counter > 0 :
                mean_cpu = mean_cpu /float(counter)
            mean_cpu = s 
            print "Cpu utilization of this time interval: "+ str(mean_cpu)
            payload = [{"cpu": mean_cpu}]            
            r = requests.post(post_url, json=payload)
            counter = 0
            mean_cpu = 0

        ##### KATANOMI GIA UPDATE CORES STO CONTAINER        
            mu, sigma = 0.75, 0.16 # mean and standard deviation
            s = np.random.normal(mu, sigma)
            print "Number of cores: "+str(s) 
            number_of_cores = s #/ float(100 )
            with open(os.devnull, 'wb') as devnull: # suppress output
                subprocess.check_call(["docker update --cpus=\"" + str(round(number_of_cores,2)) + "\" "                +str(container_name)], shell=True, stdout=devnull, stderr=subprocess.STDOUT)


if __name__ == "__main__":
        main()
