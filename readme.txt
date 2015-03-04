Clone this repo into your ~/catkin_ws/src directory 
Then cd ~/catkin_ws and run catkin_make

To run the code:
  roscore
  <new bash>
  rosrun beagleboneblack motor_comm_node.py
  
 This will run the node. 
 
To change the values sent to motor use:
 rostopic pub -r 1 beagleboneblack/MotorPower 0.1 0.1
 
 This sends 10% power to both motors and updates at a rate of 1 Hz
 
To see the motor data use:
  rostopic echo motor_data
  
To see power values sent to the motors:
  rostopic echo motor_power
