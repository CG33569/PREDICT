import sim
import math
import sys
import time
import serial

# ============================= Functions ==================================
# Function(s) to convert
def listToString(s):

    # initialize an empty string
    str1 = ""

    # traverse in the string
    for ele in s:
        str1 += ele

    # return string
    return str1

def convertToUnit(value):
    MAX_PWM = 2000
    MIN_PWM = 1000
    output = 0.0 
    if (value > MAX_PWM):
        output = 1.0
    elif (value > MIN_PWM):
        output = float((value-MIN_PWM))/float((MAX_PWM-MIN_PWM))
    return output

def sndUART(ser,send_message):
    send_lim = "$"
    send_end = "___"
    #send_message = "hello world"
    full_message = send_lim+send_message+send_end
    sending = list(full_message)
    for i in sending:
        ser.write(i.encode('utf-8'))
        time.sleep(0.001)

def rcvUART(ser):
    # initialize an empty string
    str1 = ""

    # decode message from patmos:
    busy = True
    message = [];
    temp = [];
    start = False
    start_i = 1
    start_check = False
    start_lim = ['.','F','P','G','A','1'];
    start_lim_n = len(start_lim)

    end_lim = "!";
    #end_lim_n = len(end_lim)

    dummy = False
    i = 0
    while i<100 and busy:
        i+=1
        # Read
        data = ser.read()
        try:
            data = data.decode('utf-8')              #decode message

            if (data==".")and(not start):
                start = True
                #print("Appending\n")

            if (data =="!") and (start_check):
                str1 = listToString(message)
                dummy = False
                i=1
                message = [];
                temp = [];
                start = False
                start_i = 1
                start_check = False
                busy = False


            if start_check:
                message.append(data)
                if (not dummy):
                    dummy = True

            if start:
                temp.append(data)
                start_i+=1
                #check beginning
                if (len(temp)==start_lim_n)and(not start_check):
                    start_check = (temp == start_lim)
                    start = False
                    temp.clear()


            time.sleep(0.001)
        except:
            time.sleep(0.001)


    # return string
    return str1
# ================= Defining the drone from Coppelia as an object =================
class ClientCoppelia:
    def __enter__(self):
        self.intSignalName='legacyRemoteApiStepCounter'
        self.stepCounter=0
        self.maxForce=100
        sim.simxFinish(-1) # just in case, close all opened connections
        self.id=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
        return self
    
    def __exit__(self,*err):
        sim.simxFinish(-1)

    def init(self):
        print('Getting objects')
        # Motors
        res,self.m1=sim.simxGetObjectHandle(self.id,'m1',sim.simx_opmode_blocking)
        res,self.m2=sim.simxGetObjectHandle(self.id,'m2',sim.simx_opmode_blocking)
        res,self.m3=sim.simxGetObjectHandle(self.id,'m3',sim.simx_opmode_blocking)
        res,self.m4=sim.simxGetObjectHandle(self.id,'m4',sim.simx_opmode_blocking)
        # Target velocities
        value = 1
        self.motorTarget = [value,value,value,value]

        # Sensors
        res,self.imu=sim.simxGetObjectHandle(self.id,'IMU_fr',sim.simx_opmode_blocking)
        res,self.baro=sim.simxGetObjectHandle(self.id,'baro_fr',sim.simx_opmode_blocking)
        res,self.gps=sim.simxGetObjectHandle(self.id,'GPS_fr',sim.simx_opmode_blocking)
        self.imuPos_old = [0, 0, 0]
        self.imuOrient_old = [0, 0, 0]
        self.altitude = 0
        self.compass = 0
        self.compass_old = 0
        self.gpsPos = [0, 0, 0]

         # Others
        self.linearVel = [0, 0, 0]
        self.linearVel_old = [0, 0, 0]
        self.linearAcc = [0, 0, 0]
        self.angularVel = [0, 0, 0]
        self.angularVel_old = [0, 0, 0]
        self.angularAcc = [0, 0, 0]
        self.MAX_VEL = 200

        # Center
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0]

        return self

    def actuation(self):
        # Remember that two motors spin CW and the other two CCW!
        sim.simxSetJointTargetVelocity(self.id, self.m1, -self.motorTarget[0], sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.id, self.m2, self.motorTarget[1], sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.id, self.m3, -self.motorTarget[2], sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.id, self.m4, self.motorTarget[3], sim.simx_opmode_oneshot)

    def sensing(self):
        res,self.imuPos=sim.simxGetObjectPosition(self.id,self.imu,-1,sim.simx_opmode_blocking)
        res,self.imuOrient = sim.simxGetObjectOrientation(self.id,self.imu,-1,sim.simx_opmode_blocking)
        res,self.baro_value = sim.simxGetObjectPosition(self.id,self.baro,-1,sim.simx_opmode_blocking)
        res,self.gps_value = sim.simxGetObjectOrientation(self.id,self.gps,-1,sim.simx_opmode_blocking)
        res,self.gps_pos = sim.simxGetObjectPosition(self.id,self.gps,-1,sim.simx_opmode_blocking)

        # Get relevant values 
        self.altitude = self.baro_value[2]
        self.compass =self.gps_value[2]
        self.gpsPos[0] = self.gps_pos[0]
        self.gpsPos[1] = self.gps_pos[1]
    
        # Calculate velocity and acceleration, linear + angular
        for i in range(3):
            self.linearVel[i] = self.imuPos[i] - self.imuPos_old[i]
            self.angularVel[i] = self.imuOrient[i] - self.imuOrient_old[i]

            self.linearAcc[i] = self.linearVel[i] - self.linearVel_old[i]
            self.angularAcc[i] = self.angularVel[i] - self.angularVel_old[i]

        # Center position and orientation
        self.position[0] = (self.imuPos[0] + self.gpsPos[0])*0.5
        self.position[1] = (self.imuPos[1] + self.gpsPos[1])*0.5
        self.position[2] = (self.imuPos[2] + self.altitude)*0.5
        self.orientation[0] = self.imuOrient[0]
        self.orientation[1] = self.imuOrient[2]
        self.orientation[2] = (self.imuOrient[2] + self.compass)*0.5

        # Update
        self.imuPos_old = self.imuPos
        self.imuOrient_old = self.imuOrient
        self.linearVel_old = self.linearVel
        self.angularVel_old = self.angularVel

# ============================= main =============================
print('FPGA communication start up')
serUSB = serial.Serial(port = "/dev/ttyUSB1", baudrate=115200, bytesize=8, timeout=0.2, stopbits=serial.STOPBITS_ONE)
serUSB.reset_output_buffer()
serUSB.reset_input_buffer()
# FPGA - Base variables
j = 0
from_patmos = ""
to_patmos = ""
patmos_motor = [1000, 1000, 1000, 1000]
END_CMD = 'END_SIM'

# Start communication with the simulator
with ClientCoppelia() as drone:
    if drone.id!=-1:
        print ('Connected to Coppelia Simulator')        
        # Start streaming drone.intSignalName integer signal, that signals when a step is finished:
        sim.simxGetIntegerSignal(drone.id,drone.intSignalName,sim.simx_opmode_streaming)
        drone.init()

        # ---> main loop <---
        while(True):
            # 1. Get info from the sensors
            drone.sensing()

            # 2. Send info to FPGA
            to_patmos = ''
            # The message format is fixed to have +/- signed and 4 digits
            for i in range(3):
                temp = int(drone.position[i]*1000)
                temp_abs = abs(temp)
                if(temp > 0):
                    to_patmos += '+'
                else:
                    to_patmos += '-'

                if(temp_abs<1000):
                    to_patmos += '0'
                elif(temp_abs<100):
                    to_patmos += '0'
                elif(temp_abs<10):
                    to_patmos += '0'
                to_patmos += str(temp_abs) + ';'
            print(to_patmos) 
            sndUART(serUSB,to_patmos)

            # 2. Get motors cmd from FPGA
            from_patmos = rcvUART(serUSB)
            command = from_patmos.split(';')

            # 3. Send motors cmd to simulator
            if(command[0] != END_CMD):
                for i in range(4):
                    patmos_motor[i] = int(command[i+3])
                    temp_value = convertToUnit(patmos_motor[i])
                    drone.motorTarget[i] = int(temp_value * drone.MAX_VEL)
                drone.actuation()  
                print('Drone motors velocity = ' + str(drone.motorTarget))
            else:
                print('Received end command')
                break

            # loop handling
            #time.sleep(0.1)
            j+=1
print('Program finished')
