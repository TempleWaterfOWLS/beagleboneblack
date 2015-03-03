#!/usr/bin/env python

# Node to interface motor controllers to ROS
# Looks for motor power and motor node number from ROS 
# Outputs the motor response information to ROS

from motor_comm import *
import rospy
from beagleboneblack.msg import MotorPower
from beagleboneblack.msg import MotorResponse


def power_level(data):
  '''
  Function to send power levels to motor 
  '''
  motors.set_thrust(data.power1,data.power2)
  motors.send_motors_power_level()
  

def motor_response_to_ros(motors, pub):
  '''
  Function to take motor response and put it on ROS
  '''
  motor_data = MotorResponse()
  motor_data.motor_id = motors.response[1]
  motor_data.rpm = motors.response[7]
  motor_data.bus_voltage = motors.response[8]
  motor_data.bus_current = motors.response[9]
  motor_data.temperature = motors.response[10]
  motor_data.fault_flag = motors.response[11]
  
  pub.publish(motor_data)    
  motors.toggle_node_id()
  
  
def motor_node():
  '''
  Top level function to handle connection of motors with ROS
  '''
  motors = motor_comm()
<<<<<<< HEAD
  
=======
    
>>>>>>> 3b9af6d0ed05f48b463531da5208cc74866eb8d6
  pub = rospy.Publisher('motor_data', MotorResponse, queue_size=10)
  rospy.init_node('motor_comm')
  rospy.Subscriber("motor_power", MotorPower, power_level)
  rate = rospy.Rate(10)
  
  #spins at 10 Hz and puts the motors response on ROS
  while not rospy.is_shutdown():
    motors.send_motors_power_level()
    motor_response_to_ros(motors, pub)
    rate.sleep()
    


if __name__ == '__main__':
  try: 
    motor_node()
  except rospy.ROSInterruptException:
    pass
