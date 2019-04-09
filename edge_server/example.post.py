#!/usr/bin/env python
import requests
import json
import time
import sys

CELL_SIZE = 50 
def main():
    a=[0,2]

    post_url = "http://0.0.0.0:8000/dijkstra"
    i = int(a[0])
    j = int(a[1])
    x = j * CELL_SIZE + CELL_SIZE/2 
    y = i * CELL_SIZE + CELL_SIZE/2 
    payload = [ {
            "i": int(i) ,# 6 
            "j": int(j) , # 7 
            "x": int(x) , # 150
            "y": int(y) , # 175
            "dest": 1 ,
            "or": 170
            }]
    r = requests.post(post_url, json=payload )# , data=json)
    a = json.loads(r.text)
    print a  
if __name__ == "__main__":
        main()
