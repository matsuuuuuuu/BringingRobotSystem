import time
import RPIservo

scGear = RPIservo.ServoCtrl()
scGear.moveInit()

arm13 = RPIservo.ServoCtrl()
arm13.start()
arm14 = RPIservo.ServoCtrl()
arm14.start()
arm15 = RPIservo.ServoCtrl()
arm15.start()

init_pwm0 = scGear.initPos[0]
init_pwm1 = scGear.initPos[1]
init_pwm2 = scGear.initPos[2]
init_pwm3 = scGear.initPos[3]

def servoPosInit():
    scGear.initConfig(0,init_pwm0,1)
    arm13.initConfig(1,init_pwm1,1)
    arm14.initConfig(2,init_pwm2,1)
    arm15.initConfig(3,init_pwm3,1)
    
if __name__ == '__main__':
	servoPosInit()
	time.sleep(1.0)
	#arm13.singleServo(13, -1, 3)
	#time.sleep(5.0)
	arm13.singleServo(13, 1, 3)
	time.sleep(1.0)
	arm15.singleServo(15, 1, 3)
	time.sleep(2.0)
	arm13.singleServo(13, -1, 3)
	time.sleep(2.0)
	#arm13.stopWiggle()
	
	#arm14.singleServo(14, 1, 3)
	

