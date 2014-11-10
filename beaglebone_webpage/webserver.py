''' 
Web Server Application meant to run on beagleboard for control of a robot over WLAN
Creates website as front-end for submitting thrust data to motors on robot
Created by Zack Smith 10/31/14
'''
# Imports for website generation
from flask import *
import os
# Imports for motor integration - use thread to spawn threads and adjust global motor vals
from threading import Thread
import time
from motor_comm import *
M1 = -.1; M2 = -.1
# Create instance of app
app = Flask(__name__)
app.secret_key = os.urandom(24)

# Function to set up main page and motor logic
@app.route('/', methods=['GET','POST'])
def main_page():
    # Initialize session dictionary
    session['error'] = False
    if 'stop' not in session.keys():
        session['stop'] = True
    if 'thread' not in session.keys():
        session['thread'] = False
    if 'M1' not in session.keys():
        session['M1'] = 0.0
    if 'M2' not in session.keys():
        session['M2'] = 0.0
    hold_val = [session['M1'], session['M2']]
    # Check for post response
    if request.method == 'POST':
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
        elif 'M2' in request.form.keys() and not session['stop']:
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
        elif 'stop' in request.form.keys():
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
'''
def spawn_threads():
    while 1:
        print "Spawning thread..."
        child_thread = Thread(target=print_thrust)
        child_thread.daemon = True
        child_thread.start()
        time.sleep(5)
'''
def print_thrust():
    #create object of class motor_comm
    motors = motor_comm()
    while not isinstance(M1,int) and not isinstance(M2,int):
        if (M1 == -.1 and M2 == -.1):
            continue
        print "Set thrust M1: "+ str(M1)+" M2: "+str(M2)
        motors.set_thrust(M1,M2)
        print "Sending motor command"
        motors.send_motors_power_level()
        print "Print Thrust"
        print M1, M2
        print type(M1), type(M2)
        print "Ending thrust..."
        time.sleep(.25)
        print "execution finish"

if __name__ == '__main__':    
    t = Thread(target=print_thrust)
    t.daemon = True
    t.start()
    app.run(debug=True)
