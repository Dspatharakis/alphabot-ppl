#!/usr/bin/env python
import requests
import json 

def main():
    
    post_url = "http://192.168.1.114:8000/dijkstra"
    #files  = {"file": open("./images/candidate1450.jpg", "rb")}
    #r = requests.post(post_url, files=files )# , data=json)
    payload = [ {
            "i": 3 ,
            "j": 0 ,
            "x": 1 , 
            "y": 139 , 
            "or": 170
            }]
    
            
    r = requests.post(post_url, json=payload )# , data=json)
    print (r.text)
    a = json.loads(r.text)
    print (a[0])
    print (a[1])

    #a = json.loads(r.text)
    #print a[2] 
if __name__ == "__main__":
        main()
