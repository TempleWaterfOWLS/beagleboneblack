''' 
Web Server Application meant to run on beagleboard for control of a robot over WLAN
Creates website as front-end for submitting thrust data to motors on robot
Created by Zack Smith 10/31/14
'''
# Imports for website generation
from flask import *
import os
# Imports for motor integration
from multiprocessing import Process
import time
# import motor_control as mc

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

    # Check for post response
    if request.method == 'POST':
        # Parse request keys for info
        if 'M1' in request.form.keys() :
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
        elif 'M2' in request.form.keys():
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

        # Got unexpected POST
        else:
            print "Unhandled POST keys:"
            print request.form.keys()

    # Print debugging info
    print "-----------------------------------"
    print "VALUES IN SESSION DICT"
    for keys in session.keys():
        print 'Key: ' + keys + ' Value: ' + str(session[keys])
    # Return Template
    return render_template('backup.html',session=session)

if __name__ == '__main__':    
    app.run(debug=True)














