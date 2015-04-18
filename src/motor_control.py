#!/usr/bin/env python
'''
  ROS node to control the motors
  The motors can only be sent a power percent. This node allows for control
   of RPM as well as looking for fault conditions such as stall and over heat.
'''
from pid import PID
from beagleboneblack.msg import MotorPower
from beagleboneblack.msg import MotorData
from beagleboneblack.msg import MotorRPM

class motor_control():
  def __init__(self): 
    #pid loop for each motor
    self.pid0=PID(0.0001)
    self.pid1=PID(0.0001)
      
    self.pub = rospy.Publisher('motor_power', MotorPower, queue_size=10)
    
    self.motor_power=MotorPower()
    self.motor_power.power1=0
    self.motor_power.power2=0
    
  def set_motor_power(self):
    self.motor_power.power1=self.pid0.PID+self.motor_power.power1
    self.motor_power.power2=self.pid1.PID+self.motor_power.power2

def set_rpm(data,control):  
  control.pid0.setPoint(data.rpm0)
  control.pid1.setPoint(data.rpm1)

def get_data(data,control):
  '''
  Put motor response data into control object
  '''
  if data.motor_id == 0:
    control.data0=data
  elif data.motor_id==1:
    control.data1=data
  else
    control.error=data
  
def motor_control_node():
  '''
  Top level function to handle control of motors with ROS
  Takes in MotorRPM
  Outputs MotorPower
  '''
  control=motor_control()
  rospy.init_node('motor_control')
  rate = rospy.Rate(5)
  
  while not rospy.is_shutdown(): 
    #incase the motors haven't been turned on yet
    try:
      control.pid0.update(control.data0.rpm)
      control.pid1.update(control.data1.rpm)
    except AttributeError:
      control.pid0.update(0)
      control.pid1.update(0)
      
    pub.publish(control.motor_power)
    
    rospy.Subscriber("motor_rpm", MotorRPM,set_rpm,control)
    rospy.subscriber("motor_data", MotorResponse,get_data,control)
  

if __name__ == '__main__':
  try: 
    motor_control_node()
  except rospy.ROSInterruptException:
    pass
