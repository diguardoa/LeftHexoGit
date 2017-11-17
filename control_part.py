#!/usr/bin/env python
# Written by Antonio Di Guardo


## Simple talker demo that publishsed std_msgs/Strings messages
## to the 'chatter' topica

import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_I2C import Adafruit_I2C
import Adafruit_BBIO.ADC as ADC
 
ADC.setup()
#i2c = Adafruit_I2C(0x77)

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

# Variable
Kp = 0.02 #inverso costante molla virtuale
loop_rate = 100
DEG_TO_DUTY = 0.059 # empiric
LOW_LIMIT = 4.6
UPP_LIMIT = 13.45
ZERO_MOTOR = 9 # to be set

ZERO_ADC = 0.54
SCALE_FACTOR = 100

# Pulsante 1
GPIO.setup("P9_12", GPIO.IN)
#GPIO.output("GPIO0_26", GPIO.HIGH)

#GPIO.cleanup()
#PWM.start(channel, duty, freq=2000, polarity=0)
#PWM.start("P9_14", 50)
 
#optionally, you can set the frequency as well as the polarity from their defaults:

PWM.start("P9_14",ZERO_MOTOR,60,0)

force_reference = 0
position_reference = 0

def move_servo(deg):
    duty = ZERO_MOTOR + DEG_TO_DUTY*deg
    if duty < LOW_LIMIT:
        duty = LOW_LIMIT	
    elif duty > UPP_LIMIT:
        duty = UPP_LIMIT
    
#    rospy.loginfo("Duty cycle %s", duty)
    PWM.set_duty_cycle("P9_14", float(duty))
	

def callback_force(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    force_reference = data.data

def callback_stiff(data):
    global Kp
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    Kp = data.data
	
def callback_position(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    servo_reference = data.data
    move_servo(servo_reference)
#    PWM.set_frequency("P9_14", float(servo_reference))
	
def talker():
    global Kp
	
	# Configure Publishers
    send_force = rospy.Publisher('force_curr', Float64, queue_size=10)
    send_position = rospy.Publisher('position_curr', Float64, queue_size=10)
#    send_stiff = rospy.Publisher('stiffness_get', Float64, queue_size=10)

    rospy.init_node('left_hexo_hand', anonymous=True)
	
	# Configure Subscrivers
    rospy.Subscriber('force_ref', Float64, callback_force)
    rospy.Subscriber('position_ref', Float64, callback_position)
    rospy.Subscriber('stiffness_set', Float64, callback_stiff)
	
    rate = rospy.Rate(loop_rate) # hz
	
    while not rospy.is_shutdown():
# Read Data from force ADC
        force_volt = ADC.read("AIN1")
        force_sensor = (force_volt - ZERO_ADC) * SCALE_FACTOR

# Control loop on the motor
        error = force_reference - force_sensor
        ref_pos = Kp * error
#        send_stiff.publish(Kp)
# Send Position reference to the motor
        move_servo(ref_pos)
		
# Send Exo Force
        force_msg = force_sensor
        send_force.publish(force_msg)
# Send Exo Position
        position_msg = ref_pos
        send_position.publish(position_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
