import RPi.GPIO as GPIO
import time
import threading
import numpy
import matplotlib.pyplot as plt

rcounter=0
lcounter=0

class PID:
    """PID Controller
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, speed=2.8, duty=30):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.err_pre = 0
        self.err_last = 0
        self.u = 0
        self.integral = 0
        self.ideal_speed = speed
        self.last_duty = duty
        self.pre_duty = duty

	
    def update(self,feedback_value):
        print("TARGET:",self.ideal_speed)
        self.err_pre = self.ideal_speed - feedback_value
        self.integral+= self.err_pre
        self.u = self.Kp*self.err_pre + self.Ki*self.integral + self.Kd*(self.err_pre-self.err_last)
        self.err_last = self.err_pre
        self.pre_duty = self.last_duty + self.u
        if self.pre_duty > 100:
            self.pre_duty = 100
        elif self.pre_duty < 0:
            self.pre_duty = 0
        self.last_duty = self.pre_duty
        return self.pre_duty

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain
		

def my_callback(channel):
    global lcounter
    global rcounter
    EA, I2, I1, EB, I4, I3, LS, RS = (13, 19, 26, 16, 20, 21, 6, 12)
    if (channel==LS):
        lcounter+=1
    elif(channel==RS):
        rcounter+=1
            
def getspeed():
    global rspeed
    global lspeed
    global lcounter
    global rcounter

    ################ADD##################
    EA, I2, I1, EB, I4, I3, LS, RS = (13, 19, 26, 16, 20, 21, 6, 12)
    FREQUENCY = 50
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([EA, I2, I1, EB, I4, I3], GPIO.OUT)
    GPIO.setup([LS, RS],GPIO.IN)
    GPIO.output([EA, I2, EB, I3], GPIO.LOW)
    GPIO.output([I1, I4], GPIO.HIGH)
    
    GPIO.add_event_detect(LS,GPIO.RISING,callback=my_callback)
    GPIO.add_event_detect(RS,GPIO.RISING,callback=my_callback)
    #####################################
    
    while True:
        rspeed=(rcounter/3.00)
        lspeed=(lcounter/3.00)
        rcounter = 0
        lcounter = 0
        time.sleep(0.3)
   

   
thread1=threading.Thread(target=getspeed)
thread1.start()
	
	

class PID_control(threading.Thread):
    def __init__(self,speed_l,speed_r):
        threading.Thread.__init__(self)
        self.speedl=speed_l
        self.speedr=speed_r

  
        EA, I2, I1, EB, I4, I3, LS, RS = (13, 19, 26, 16, 20, 21, 6, 12)
        FREQUENCY = 50
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([EA, I2, I1, EB, I4, I3], GPIO.OUT)
        GPIO.setup([LS, RS],GPIO.IN)
        GPIO.output([EA, I2, EB, I3], GPIO.LOW)
        GPIO.output([I1, I4], GPIO.HIGH)

        self.pwma = GPIO.PWM(EA, FREQUENCY)
        self.pwmb = GPIO.PWM(EB, FREQUENCY)
        self.pwma.start(0)
        self.pwmb.start(0)

        lspeed = 0
        rspeed = 0
        lcounter = 0
        rcounter = 0
        

        self.l_origin_duty = 3
        self.r_origin_duty = 5
        self.pwma.start(self.l_origin_duty)
        self.pwmb.start(self.r_origin_duty)
        print("pid thread init")

    
    def run(self):
        i=0
        delta=0.05
        print("pid thread started")
        try:
            L_control = PID(1,0.3,1,0,self.l_origin_duty)            #8#2.5 0.4 6           #4#1,0.3,3.5
            R_control = PID(1,0.3,1,0,self.r_origin_duty)           #8#2.5 0.4 6         #4#1,0.3,3
        
            while True:

                if self.speedl>=delta:
                    self.speedl=self.speedl-delta
                if self.speedr>=delta:
                    self.speedr=self.speedr-delta
                L_control.ideal_speed=self.speedl
                R_control.ideal_speed=self.speedr
                #tmpl=L_control.update(lspeed)
                #tmpr=R_control.update(rspeed)
                tmpl=self.speedl/2.0*100
                tmpr=self.speedr/2.0*100
                print("output:",tmpl,tmpr)
                
                self.pwma.ChangeDutyCycle(tmpl)
                self.pwmb.ChangeDutyCycle(tmpr)
                time.sleep(0.3)
                i+=  1
        #print ('left: %f  right: %f lduty: %f rduty: %f'%(lspeed,rspeed,L_control.pre_duty,R_control.pre_duty))
        
        except KeyboardInterrupt:
            pass
        self.pwma.stop()
        self.pwmb.stop()
    
        GPIO.cleanup()
    
