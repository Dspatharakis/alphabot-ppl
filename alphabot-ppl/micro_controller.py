import RPi.GPIO as GPIO
import time
import sys
import signal
from math import pi, fabs, cos, sin 
from alphabot import AlphaBot
import multiprocessing

R = 0.034 # wheel radius (m) 6,6 cm 
r = 0.0165  #wood wheel radious
L = 0.132   # distance between  wheels 
Ab = AlphaBot()
Ab.stop()
vmax = 100
vmin= 30

#for wr the equation for voltage is v = (wr - B)/ A
A = 0.154085#0.16136027
B = 6.8064#-1.2869597
#for wl the equation for voltage is v = (wl - D)/ C
C = 0.166486#0.16225248
D = 5.55511#-0.3943191

F = 7.092
G = 318.5

class MicroControler(object) :

    def move_and_control(self, a):
        start_time = time.time()
        T = 0.3 
        reference_position = a
        xo = a[0]
        yo = a[1]
        fo = a[2]
        xref = a[3]
        yref = a[4]
        fref = int(a[5])
        orientation = 0 
        rotational = 0 
        e = 0.2
        vmax = 100
        vmin= 20

        if fref!=fo:
            rotational = 1
            #TODO uncomment
            #T = ((fabs(fref)+6.2687)/134.7328)
            T = ((fabs(fref)+F)/G)

        fref = fref*pi / 180
        counter = 2
        moves = 0
        
        dt = T
        while moves<100:
            moves += 1
            # set velocities to reach xref,yref,fref
            # prwta peristrofikh meta metaforikh
            if fabs(fabs(fref)-fabs(fo))>= e  :
                if rotational == 1 and moves > 1:
                    dt = ((fabs(fabs(fref*180/pi)-fabs(fo*180/pi))+F)/G)
                elif rotational ==0:
                    dt = ((fabs(fabs(fref*180/pi)-fabs(fo*180/pi))+F)/G)
                #    dt = ((fabs(fabs(fref*180/pi)-fabs(fo*180/pi))+6.2687)/134.7328)
                w = L*(fref-fo)/(dt*2*R) # xronos misos diplasio w
                a = fabs(w) / w
		w = 12 
		right_voltage = (w - B)/A
                left_voltage =  (w - D)/C
		w = w* a
		
                #print ("right_voltage: "+str(right_voltage)+ " left_voltage: "+str(left_voltage))
                if w < 0:  #prepei na stripsw pros ta deksia ( + , + )
                    orientation = 2
                    right_voltage= right_voltage
                    left_voltage= left_voltage
                else: # prepei na stripsw pros ta aristera ( - , - )
                    orientation = 3
                    right_voltage= -right_voltage
                    left_voltage= -left_voltage
                print ("right_voltage: "+str(right_voltage)+ " left_voltage: "+str(left_voltage))
            
            elif (fabs(fabs(xref)-fabs(xo))>= 0.05 and (rotational!= 1)):

                #print ("metaforiki kinhsh")

                w = (xref-xo)/(R*dt)
                while True:
                    if 0 <=w< 12 : 
                        #dt = float(dt)/float(1.01)
                        w = 12 # (xref-xo)/(R*dt)
                    elif w >14: #21.5 : 
                        #dt = dt * 1.01
                        w = 14 #21.5 # (xref-xo)/(R*dt)
                    elif 0>w>-12 : 
                        #dt = dt / float(1.01)
                        w = -12 #(xref-xo)/(R*dt)
                    elif w < - 14 :
                        #dt = dt * 1.01
                        w = -14 #(xref-xo)/(R*dt)
                    else: break 
                print w 
                right_voltage = (abs(w) - B)/A
                left_voltage =  (abs(w) - D)/C
                

                if w >= 0:
                    orientation = 0
                    right_voltage = -right_voltage # an w > 0 tote prepei na paw eutheia ara ( - , + )
                    left_voltage = +left_voltage
                if w < 0:         # an w < 0 tote prepei na paw opisthen ara ( + , - )            
                    right_voltage = right_voltage 
                    left_voltage =  -left_voltage
                    orientation = 1 
            
            else : 
                Ab.stop()
                #print ("STOPPPPPPPPPPPPPPPPPPPP")
                break
            print ("To dt einai auth th fora : ")+str(dt)
            #print ("To w einai : ")+str(w)
            print ("right voltage :" + str(right_voltage) , "left_voltage "+ str(left_voltage))
            #print ("apostasi apo stoxo se cm: "+str(fabs(xref-xo)) )
            Ab.setMotor(right_voltage, left_voltage) # PRWTA RIGHT META LEFT !!! EINAI ANAPODA STHN SETMOTOR
            #fork processes and grab results after dt
            manager = multiprocessing.Manager()
            return_dict = manager.dict()
            jobs = []
            p = multiprocessing.Process(target=self.right_velocity,args=(0,  return_dict))
            jobs.append(p)
            p.daemon=True
            p.start()
            k = multiprocessing.Process(target=self.left_velocity,args = ( 1 , return_dict))
            jobs.append(k)
            k.daemon=True
            k.start()
            time.sleep(dt)
            p.terminate()
            k.terminate()
            p.join(T)       
            k.join(T)
            # try to grab measurements
            counter = 2
            try:
                wr = return_dict[0]
                dtr = return_dict[2]
                #TODO
                if wr > A*vmax+B :
                    print "measured maximum wr "
                    wr = A * vmax +B
            except: 
                #print ("error1")
                wr = pi /(10 * T)  
                #wr = 0 
                #dtr = dt
                counter = counter -1 
            try:
                wl = return_dict[1]
                dtl = return_dict[3]
                #TODO
                if wl > C*vmax+D :
                    print "measured maximum wl "
                    wl = C * vmax +D
            except: 
                #print ("error2")
                wl = pi / (10 * T) 
                #wl = 0 
                counter = counter -1 
                dtl = dt
            if counter == 2 :
                #dt = (dtl+dtr)/counter
                print "ok"
            elif counter==0 and rotational==1:
                print ("two errors when reading from light sensors")
                #wr = (fref-fo)*L/(2*R*dt) # xronos misos diplasio w
                #wl = wr 
                #break  
                #wr = 0 
                #wl = 0 
            #if wr == 0 and wl != 0 :
            #    wr = wl 
            #    dtr= dtl
            #    dt = (dtl+dtr)/2
            #elif wl == 0 and wr != 0 :
            #    wl = wr
            #    dtl = dtr
            #    dt = (dtl+dtr)/2
         
        #calculate new xo,yo,fo
            if orientation == 3: # tou eipa na stripsei aristera epitopou 
                wr = wr    # eixa paei prin deksia sta - kai twra thelw na paw aristera sta +
                wl = -wl   # ara wr - wl prepei na einai thetiko 
            if orientation == 2: # tou eipa na stripsei deksia epitopou
                wr = -wr   # eixa paei prin aristera sta + kai twra thelw na paw deksia sta -  
                wl =  wl   # ara wr-wl  prepei na einai arnhtiko
             
            if orientation == 1 :
                wr= -wr
                wl= -wl
	    if (dt * R*(wr-wl)/L)> 0.7:
		c = 0.78
	    else:
                c = 1 
            fo = fo + (dt * R*(wr-wl)/L)/c
            print ("Measured  wr: "+str(round(wr,2))+" rad/s" , "  wl: "+str(round(wl,2))+" rad/s", " dt of mesaurement: "+ str(round(dt,2)) +" ms",  " Angle from initial orientation of this step, f0: "+ str(round((fabs(fref*180/pi)-fabs((fo*180/pi))),2))+" degrees")
            
            #if fo > 2*pi : 
            #    fo = fo - 2*pi
            #    print "exw kanei ena kuklo"
                # estw oti exw kanei kuklo mhdenise
            #if fo < -2*pi : 
            #    fo = fo + 2*pi
            #    print "exw kanei ena kuklo"
            
                #print ("phgainw opisthen")
            if orientation ==0  or orientation==1:
                xo = xo + (dt*cos(fo)*R*(wr+wl)/2)/0.7 
                yo = yo + dt*sin(fo)*R*(wr+wl)/2 
            print ("Measured values from light sensors:  xo: "+str(round(xo*100,2))+" cm", " yo: "+str(round(yo*100,2)) +" cm",  " fo: "+str(round((fo*180/pi),2))+" degrees")

            Ab.stop()
            #TODO edw thelei 0.5 sleep
            time.sleep(1.5)
        
        Ab.stop()
        xo = round(xo*100,2)
        yo = round(yo*100,2)
        fo = round(fo *180 / pi,2) 
        return xo,yo,fo
        

    def right_velocity(self, procnum, return_dict):
        DR = 8
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
        

        prev_status= 0
        changes = 0
        prev_time = time.time()
        while True:
            DR_status = GPIO.input(DR) 
            if(DR_status == 0):
                if DR_status != prev_status:
                    prev_status = 0 
                    changes += 1
                    now_time = time.time()
                    dtime= now_time - prev_time
                    w = (pi*changes)/(10* dtime) # s^2i
                    return_dict[procnum] = w #linear_velocity_right
                    return_dict[2] = dtime #linear_velocity_right
            elif (DR_status == 1):
                if DR_status != prev_status:
                    prev_status = 1

    def left_velocity(self, procnum, return_dict):
        DR = 7 
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
        
        prev_status= 0
        changes = 0
        prev_time = time.time()
        while True:
            DR_status = GPIO.input(DR)
            if(DR_status == 0):
                if DR_status != prev_status:
                    prev_status = 0 
                    changes += 1
                    now_time = time.time()
                    dtime= now_time - prev_time
                    w = (pi*changes)/(10* dtime) # s^2
                    return_dict[procnum] = w #linear_velocity_right
                    return_dict[3] = dtime #linear_velocity_right
            elif (DR_status == 1):
                if DR_status != prev_status:
                    prev_status = 1




