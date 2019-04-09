#!/usr/bin/env python
import os
import requests
import json
import time
import sys
import subprocess
import csv
import json

def main():
    post_url = "http://0.0.0.0:8000/"
    for a in range (25 , 825, 25):
        number_of_cores = a / float(100 )
	art = 0 
        with open(os.devnull, 'wb') as devnull: # suppress output
            subprocess.check_call(["docker update --cpus=\"" + str(round(number_of_cores,2)) + "\" " +
            str("f3db6b9131ee ")], shell=True, stdout=devnull, stderr=subprocess.STDOUT)
            for i in range (1,24):
                start_time = time.time()
                payload = { "time" : time.time()}
                files = [("time", ("datas", json.dumps(payload), "application/json")),("file", ("temp", open('./dataset/' + str(i) + ".jpg", "rb"),'application/octet-stream'))]
                #files = {"file": open('./dataset/' + str(i) + ".jpg", "rb")}
                r = requests.post(post_url, files=files)
		end_time = time.time()
		response_time = end_time - start_time
		art += response_time
            print a 
    	    art = art / 23
    	    filename = "./statsclient"
	    with open(filename, 'a') as myfile:
	        wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
	    # If opened for the first time, insert header row           
	        if os.path.getsize(filename) == 0:
	            wr.writerow(["CPU","average_per_photo"])
	        wr.writerow([number_of_cores,art])
if __name__ == "__main__":
        main()
