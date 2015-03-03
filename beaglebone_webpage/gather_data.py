''' 
Code written to ping the BeagleBone-Based webserver for information. 
Intended to recover information from the server, which indicates
RPM data, and possibly other data ( I have no idea what the motor controllers return)
INPUTS: File name
Outputs: Writes data to text file specified by command line args Written by Zack Smith 10/01/14, modified for BBB on 2-22-14
USAGE: Python gather_data.py yoloswag.txt
Make sure you are connected to the yoloswag network before running!
'''
# Imports
import requests
import sys
from time import strftime, sleep
# Function designed to take input hex data and write to motor_hex.txt, along with timestamp formatted Year-month-day hour:minute:second
def append_to_file(hex_data):
        with open(sys.argv[1], 'a') as data_file:
                data_file.write("Data Request at time: ")
                data_file.write(strftime("%Y-%m-%d %H:%M:%S \n"))
                data_file.write(str(hex_data) + '\n')
        return
        
# Function designed to parse response for motor information
def parse_response(get_response):
	return get_response.text

# Function designed to print information about the get response for debugging purposes
def print_response(get_response):
	print "URL Accesed"
	print get_response.url
	print "Status Code:"
	print get_response.status_code
	print "Headers:"
	print get_response.headers
	print "Text content:"
	print get_response.text
        return

def main():
	# Declare serverIP, port, and desired index to create URL
	serverIP = "192.168.0.103"; port="80"; directory = "/pythoninfo";
	url="http://"+serverIP+":"+port+directory
        # Continually send requests
        while True:
		print "Getting Response..."
                get_response = requests.get(url)
                # Print out useful debugging information about the request
                # print_response(get_response)
                # Parse response for hex data
		print "Parsing Response..."
                hex_data = parse_response(get_response)
                # Write hex data to file with time stamp
                print "Appending to file..."
		append_to_file(hex_data)
                # Sleep to not overload server with requests
              	print "Sleeping for 300ms"
		sleep(0.3)

# Call main boiler plate
if __name__ == '__main__':
        main()
