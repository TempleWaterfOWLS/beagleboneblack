
import serial
import struct
import sys
import operator
import argparse
import binascii

#for BeagleBoneBlack
import Adafruit_BBIO.UART as UART

import time

#VRCSR protocol defines  
SYNC_REQUEST  =  0x5FF5
SYNC_RESPONSE =  0x0FF0
PROTOCOL_VRCSR_HEADER_SIZE = 6
PROTOCOL_VRCSR_XSUM_SIZE   = 4

#CSR Address for sending an application specific custom command
ADDR_CUSTOM_COMMAND  = 0xF0

#The command to send.
#The Propulsion command has a payload format of:
# 0xAA R_ID THRUST_0 THRUST_1 THRUST_2 ... THRUST_N 
# Where:
#   0xAA is the command byte
#   R_ID is the NODE ID of the thruster to respond with data
#   THRUST_X is the thruster power value (-1 to 1) for the thruster with motor id X
PROPULSION_COMMAND   = 0xAA

#flag for the standard thruster response which contains 
RESPONSE_THRUSTER_STANDARD = 0x2
#standard response is the device type followed by 4 32-bit floats and 1 byte
RESPONSE_THRUSTER_STANDARD_LENGTH = 1 + 4 * 4 + 1 

#The proppulsion command packets are typically sent as a multicast to a group ID defined for thrusters
THRUSTER_GROUP_ID    = 0x81

def main(): 

    #Parse command line arguments for portname, node id, motor id, and thrust values
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--com', help='comm port', default = 'COM2', dest='portname')
    parser.add_argument('-i', '--id', help="node id for the request packet", default = THRUSTER_GROUP_ID, dest='node_id')
    parser.add_argument('-m', '--motor', help="motor NODE ID from which to get a response", default = 0, dest='motor_id')
    parser.add_argument('thrust', metavar='N', type=float, nargs='*', help='list of thrust settings, in order of motor id' )
    args = parser.parse_args()

    #default to 0 thrust for motor 0 if no thrust parameters are passed in
    if (len(args.thrust) == 0):
        thrust  = [0.0]
    else:
        thrust = args.thrust

    #open the serial port
    UART.setup("UART4")
    try:        
        port = serial.Serial(port = "/dev/ttyO4",115200)
        port.timeout = 1
    except IOError:
        print ("Error:  Could not open serial port: " + args.portname)     
        sys.exit()

        
    #Create the custom command packet for setting the power level to a group of thrusters
    #generate the header
    flag = RESPONSE_THRUSTER_STANDARD
    CSR_address = ADDR_CUSTOM_COMMAND
    length = 2 + len(thrust) * 4
    header = bytearray(struct.pack('HBBBB',SYNC_REQUEST,int(args.node_id),flag,CSR_address,length))
    header_checksum = bytearray(struct.pack('I', binascii.crc32(header))) 

    #generate the paylaod, limiting the thrust to reasonable values
    payload = bytearray(struct.pack('BB', PROPULSION_COMMAND, int(args.motor_id)))
    for t in thrust:
        t = max(t,-1)
        t = min(t, 1)
        payload += bytearray(struct.pack('f',t))
	
  #  print (binascii.crc32(payload))
    payload_checksum = bytearray(struct.pack('I', binascii.crc32(payload)))    

    #send the packet and wait for a response
    packet = header + header_checksum + payload + payload_checksum 

    #put the packet on the wire
    t=time.time()
    port.write(bytes(packet))


    #get the response

    expected_response_length = PROTOCOL_VRCSR_HEADER_SIZE + PROTOCOL_VRCSR_XSUM_SIZE +  RESPONSE_THRUSTER_STANDARD_LENGTH +  PROTOCOL_VRCSR_XSUM_SIZE
 #   print (expected_response_length)
    response_buf = port.read(expected_response_length)
    print("Elapsed time: %f" % (time.time()-t))
    print ("Got response: %d" % len(response_buf))
    #parse the response
    response = struct.unpack('=HBBBB I BffffB I', response_buf)
        
    #header data
    sync =              response[0]
    response_node_id =  response[1]
    flag =              response[2]
    CSR_address =       response[3]
    length =            response[4]
    header_checksum =   response[5]

    #response device type
    device_type      =   response[6];

    #
    rpm = response[7]
    bus_v = response[8]
    bus_i = response[9]
    temp = response[10]
    fault = response[11]

    payload_checksum = response[12]

    print ("\nResponse:")
    print ("\tSync:\t\t0x%x" % sync)
    print ("\tId:\t\t%d" % response_node_id)
    print ("\tFlag:\t\t0x%x" % flag)
    print ("\tAddress:\t0x%x" % CSR_address)
    print ("\tLength:\t\t0x%x" % length)
    print ("\t\tChecksum: 0x%x" % header_checksum)

    print ("\n\tDevice Type:\t\t0x%x" % device_type)
    print ("\tRPM:\t\t\t%f" % rpm)
    print ("\tBus Voltage (V):\t%f" % bus_v)
    print ("\tBus Current (A):\t%f" % bus_i)
    print ("\tTemp (C):\t\t%f" % temp)
    print ("\tFault:\t\t\t0x%x" % fault)

    print ("\t\tChecksum: 0x%x" % payload_checksum)



if __name__ == "__main__":
    main();

