# -*- coding: utf-8 -*-
#! /usr/bin/python

# 라즈베리파이 GPIO 패키지
import RPi.GPIO as GPIO
from time import sleep
from mpu6050 import mpu6050
import smbus
import math
import sys, termios, atexit
from select import select



#모터 상태3가지, 정지, 전진, 후진상태
STOP  = 0
FORWARD  = 1
BACKWORD = 2

#motor channel 설정 ##########
CH1 = 0
CH2 = 1

#pin input&output 설정
OUTPUT = 1
INPUT = 0

########### PIN 설정 ###########
ON = 1
OFF = 0

########### PWM PIN ###########
ENA = 26  #37 pin
ENB = 0   #27 pin

########### GPIO PIN ###########
IN1 = 21  #40 pin
IN2 = 20  #38 pin
IN3 = 16  #36 pin
IN4 = 32  #32 pin

########### 핀 설정 함수 ###########
def set_pin_num(EN, INA, INB):
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # 100khz 로 PWM 동작 시킴
    pwm = GPIO.PWM(EN, 100)
    # PWM 0상태에서 시작
    pwm.start(0)
    return pwm

########### 모터 제어 함수 ###########
def set_motor_val(pwm, INA, INB, speed, stat):

    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)

    if stat == FORWARD:
        GPIO.output(INA, ON)
        GPIO.output(INB, OFF)

    #뒤로
    elif stat == BACKWORD:
        GPIO.output(INA, OFF)
        GPIO.output(INB, ON)

    #정지
    elif stat == STOP:
        GPIO.output(INA, OFF)
        GPIO.output(INB, OFF)


def setMotor(ch, speed, stat):
    if ch == CH1:
        #pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        set_motor_val(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        set_motor_val(pwmB, IN3, IN4, speed, stat)


########### GPIO 모드 설정 ###########
GPIO.setmode(GPIO.BCM)

#모터 핀 설정
#핀 설정후 PWM 핸들 얻어옴
pwmA = set_pin_num(ENA, IN1, IN2)
pwmB = set_pin_num(ENB, IN3, IN4)


#mpu6050 imu센서 설정하기

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

#global tempval

def read_byte(adr):
	return bus.read_byte_data(address, adr)

def read_word(adr):
	try:
		high = bus.read_byte_data(address, adr)
		low = bus.read_byte_data(address, adr + 1)
		val = (high << 8) + low
		#tempval = val
		return val
	except:
		return 0x0000


def read_word_2c(adr):
	val = read_word(adr)

	if(val >= 0x8000):
		return -((65535 - val) + 1)

	else:
		return val

def dist(a, b):
	return math.sqrt((a * a) + (b * b))

def get_y_rotation(x, y, z):
	radians = math.atan2(x, dist(y, z))
	return math.degrees(radians)

def get_x_rotation(x, y, z):
	radians = math.atan2(y, dist(x, z))
	return math.degrees(radians)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards

address = 0x68

bus.write_byte_data(address, power_mgmt_1, 0)

def data_transform(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# save the terminal settings
fd = sys.stdin.fileno()
new_term = termios.tcgetattr(fd)
old_term = termios.tcgetattr(fd)

# new terminal setting unbuffered
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

# switch to normal terminal
def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

# switch to unbuffered terminal
def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

def putch(ch):
    sys.stdout.write(ch)

def getch():
    return sys.stdin.read(1)

def getche():
    ch = getch()
    putch(ch)
    return ch

kp = 1600.0
ki = 3600.0
kd = 50.0


P_term = 0.0
I_term = 0.0
D_term = 0.0

dT = 0.1

error = 0.0
error_prov = 0.0

output = 0.0

goal_slope = 10.5

if __name__ == '__main__':
    atexit.register(set_normal_term)
    set_curses_term()

try :

    while 1:

		P_term = 0.0
		I_term = 0.0
		D_term = 0.0
	
		accel_xout = read_word_2c(0x3b)
        	accel_yout = read_word_2c(0x3d)
        	accel_zout = read_word_2c(0x3f)

        	accel_xout_scaled = accel_xout / 16384.0
        	accel_yout_scaled = accel_yout / 16384.0
        	accel_zout_scaled = accel_zout / 16384.0

	 	x_slope = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        	y_slope = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

		error = goal_slope - y_slope

        	P_term = kp * error
        	I_term += ki * error * dT
        	D_term = kd * (error - error_prov) / dT

        	error_prov = error


        	if I_term > 1000:
                	I_term = 1000
        	elif I_term < -1000:
                	I_term = -1000

        	output = P_term + I_term + D_term


       	 	speed = output


		if speed > 0.000:

            		if speed > 10000:
                		speed = 10000
            		speed /= 100
            		setMotor(CH1, int(speed), FORWARD)
            		setMotor(CH2, int(speed), FORWARD)
        	elif speed < 0.0000:
            		if speed < -10000:
                		speed = -10000

            		speed /= 100
            		setMotor(CH1, int(-1 * speed), BACKWORD)
            		setMotor(CH2, int(-1 * speed), BACKWORD)
        	else:
            		setMotor(CH1, 0, STOP)
            		setMotor(CH2, 0, STOP)


        #sys.stdout.write('balancing')



except KeyboardInterrupt:

    # 종료
    GPIO.cleanup()


