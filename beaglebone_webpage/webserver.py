''' 
 Web Server Application meant to run on beagleboard for control of a robot over WLAN
 Creates website as front-end for submitting thrust data to motors on robot
 Created by Zack Smith 10/31/14
'''
# Imports for website generation
from flask import *
import os

#change PYTHONPATH to find motor_comm
import sys
sys.path.append("../src")

# Imports for motor integration - 
#  use thread to spawn threads and adjust global motor vals
from motor_comm import *
from threading import Thread
import time

#ROS imports
import rospy
from beagleboneblack.msg import MotorPower

pub = rospy.Publisher('motor_power', MotorPower, queue_size=10)
rospy.init_node('motor_control_website',anonymous=True)

M1 = -.0001; M2 = -.0001
# Create instance of app
app = Flask(__name__)
app.secret_key = os.urandom(24)
response_array = []
# Function to set up main page and motor logic
@app.route('/', methods=['GET','POST'])
def main_page():
    # Initialize session dictionary
    session['error'] = False
    if 'stop' not in session.keys():
        session['stop'] = False
    if 'thread' not in session.keys():
        session['thread'] = False
    if 'M1' not in session.keys():
        session['M1'] = 0.0
    if 'M2' not in session.keys():
        session['M2'] = 0.0
    hold_val = [session['M1'], session['M2']]
    # Check for post response
    print "REQUEST"
    print request.form.keys()
    if request.method == 'POST':
        print "POST DETECTED"
        # Parse request keys for info

        if 'M1' in request.form.keys() and not session['stop']:
            try:
                # Force max/min vals
                session['M1'] = float(request.form['M1'])
                if session['M1'] > 99:
                    session['M1'] = 99.0
                elif session['M1'] < -99:
                    session['M1'] = -99.0
            except ValueError:
                session['error'] = True
        # Check M2 Value
        if 'M2' in request.form.keys() and not session['stop']:
            try:
                # Force max/min vals
                session['M2'] = float(request.form['M2'])
                if session['M2'] > 99:
                    session['M2'] = 99.0
                elif session['M2'] < -99:
                    session['M2'] = -99.0
            except ValueError:
                session['error'] = True
        # Toggle stop value
        if 'stop' in request.form.keys():
            session['stop'] = not session['stop']
            if session['stop'] == False:
                hold_val[0] = session['M1']
                hold_val[1] = session['M2']
                session['M1'] = 0.0
                session['M2'] = 0.0
            else:
                session['M1'] = hold_val[0]
                session['M2'] = hold_val[1]

        # Got unexpected POST
        else:
            print "Unhandled POST keys:"
            print request.form.keys()

    # Print debugging info
    print "-----------------------------------"
    print "VALUES IN SESSION DICT"
    for keys in session.keys():
        print 'Key: ' + keys + ' Value: ' + str(session[keys])
    print "Session vars:"
    print session['M1'], session['M2']
    set_globvar_M1(session['M1'])
    set_globvar_M2(session['M2'])
    # Return Template
    return render_template('backup.html',session=session)

def set_globvar_M1(t_val):
    global M1
    M1 = t_val

def set_globvar_M2(t_val):
    global M2
    M2 = t_val

def print_thrust():
    #create object of class motor_comm
    motors = motor_comm()
    motor_power = MotorPower()
    
    node_id = 0
    while not isinstance(M1,int) and not isinstance(M2,int):
        if (M1 == -.0001 and M2 == -.0001):
            continue
        print "Set thrust M1: "+ str(M1)+" M2: "+str(M2)
        print "Sending motor command"
        motor_power.power1 = M1
        motor_power.power2 = M2
        
        #move motors
        pub.publish(motor_power)
        #motors.set_motor_response_node(node_id)
        #response=motors.send_motors_power_level()
        #response_array.append(response)
        #if (node_id==1):
        #    node_id = 0
       # else:
        #    node_id = 1
        print "Print Thrust"
        print M1, M2
        print type(M1), type(M2)
        print "Ending thrust..."
        time.sleep(.25)
        print "execution finish"

# Route to send data back to requesting thing
#Not sure if we need this anymore
@app.route('/pythoninfo', methods=['GET'])
def send_data():
    sent_data = response_array
    response_array = []
    return sent_data

if __name__ == '__main__':    
    t = Thread(target=print_thrust)
    t.daemon = True
    t.start()
    app.run(host='0.0.0.0',debug=True)
