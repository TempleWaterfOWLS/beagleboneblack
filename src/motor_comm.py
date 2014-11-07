
import serial
import struct
import sys
import operator
import argparse
import binascii

#for BeagleBoneBlack
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.GPIO as GPIO


class motor_comm():
    def self.__init__(self):
        #VRCSR protocol defines  
        self.SYNC_REQUEST  =  0x5FF5
        self.SYNC_RESPONSE =  0x0FF0
        self.PROTOCOL_VRCSR_HEADER_SIZE = 6
        self.PROTOCOL_VRCSR_XSUM_SIZE   = 4
        
        #CSR Address for sending an application specific custom command
        self.ADDR_CUSTOM_COMMAND  = 0xF0

        #The command to send.
        #The Propulsion command has a payload format of:
        # 0xAA R_ID THRUST_0 THRUST_1 THRUST_2 ... THRUST_N 
        # Where:
        #   0xAA is the command byte
        #   R_ID is the NODE ID of the thruster to respond with data
        #   THRUST_X is the thruster power value (-1 to 1) for the thruster with motor id X
        self.PROPULSION_COMMAND   = 0xAA

        #flag for the standard thruster response which contains 
        self.RESPONSE_THRUSTER_STANDARD = 0x2
        #standard response is the device type followed by 4 32-bit floats and 1 byte
        self.RESPONSE_THRUSTER_STANDARD_LENGTH = 1 + 4 * 4 + 1 

        #The proppulsion command packets are typically sent as a multicast to a group ID defined for thrusters
        self.THRUSTER_GROUP_ID    = 0x81
        
        #default to 0 thrust for motor 
        self.thrust = [0,0]
        
        #default to 0 motor node for responses
        self.motor_response_node = 0
        
        #open the serial port
        UART.setup("UART4")
        try:        
            self.port = serial.Serial(port = "/dev/ttyO4",baudrate=115200)
            self.port.timeout = 1
        except IOError:
            print ("Error:  Could not open serial port: " + args.portname)     
            sys.exit()
        
        #setup GPIO
        rec_output_enable_pin="P9_12"
        GPIO.setup(rec_output_enable_pin, GPIO.OUT)
        #set receiver output enable to enable
        GPIO.output(rec_output_enable_pin, GPIO.LOW)
        
        
    def self.set_thrust(thrust_1=self.thrust[0],thrust_2=self.thrust[1]):
    '''
    Function to set the thrust of the motors. If nothing is sent, the current thrust values are used.
    '''
        #motors take thrust levels between 1 and 0
        #scale inputs to be less than 1
        while (thrust_1 > 1):
            thrust_1=thrust_1/10
        while (thrust_2 > 1):
            thrust_2=thrust_2/10
            
        self.thrust[0]=thrust_1
        self.thrust[1]=thrust_2
        
    def self.set_motor_response_node(motor_node):
    '''
    Set motor response node
    '''
        self.motor_response_node = motor_node
    
    
    def self.send_motors_power_level():
    '''
    Sends communication to motors
    Returns response list from the motor set in self.motor_node
    '''
        #Create the custom command packet for setting the power level to a group of thrusters
        #generate the header
        flag = RESPONSE_THRUSTER_STANDARD
        CSR_address = ADDR_CUSTOM_COMMAND
        length = 2 + len(self.thrust) * 4
        header = bytearray(struct.pack('HBBBB',SYNC_REQUEST,int(self.motor_response_node),flag,CSR_address,length))
        header_checksum = bytearray(struct.pack('i', binascii.crc32(header))) 

        #generate the payload, limiting the thrust to reasonable values
        payload = bytearray(struct.pack('BB', PROPULSION_COMMAND, int(args.motor_id)))
        for t in thrust:
            t = max(t,-1)
            t = min(t, 1)
            payload += bytearray(struct.pack('f',t))
        
        payload_checksum = bytearray(struct.pack('i', binascii.crc32(payload)))    

        #send the packet and wait for a response
        packet = header + header_checksum + payload + payload_checksum 

        #put the packet on the wire
        self.port.write(bytes(packet))

        #get the response
        expected_response_length = self.PROTOCOL_VRCSR_HEADER_SIZE + self.PROTOCOL_VRCSR_XSUM_SIZE +  self.RESPONSE_THRUSTER_STANDARD_LENGTH +  self.PROTOCOL_VRCSR_XSUM_SIZE

        #read in lines from sent message 
        response_buf = self.port.read(len(bytes(packet)))
        
        #read in received lines sent from motor
        response_buf = port.read(expected_response_length)
        print ("Got response: %d" % len(response_buf))

        #parse the response
        self.response = struct.unpack('=HBBBB I BffffB I', response_buf)


    '''#header data
    sync =              response[0]
    response_node_id =  response[1]
    flag =              response[2]
    CSR_address =       response[3]
    length =            response[4]
    header_checksum =   response[5]

    #response device type
    device_type      =   response[6];

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
'''

