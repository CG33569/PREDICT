import serial
import time
import thread
import sys
# ============================= Variables ==================================
global pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll
pid_p_gain_roll = 0.45;                #Gain setting for the roll P-controller
pid_i_gain_roll = 0.0008;             #Gain setting for the roll I-controller
pid_d_gain_roll = 2.0;               #Gain setting for the roll D-controller
pid_max_roll = 400;                   #Maximum output of the PID-controller (+/-)

global pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch
pid_p_gain_pitch = pid_p_gain_roll;   #Gain setting for the pitch P-controller.
pid_i_gain_pitch = pid_i_gain_roll;   #Gain setting for the pitch I-controller.
pid_d_gain_pitch = pid_d_gain_roll;   #Gain setting for the pitch D-controller.
pid_max_pitch = 400;                  #Maximum output of the PID-controller (+/-)

global pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw
pid_p_gain_yaw = 1;                   #Gain setting for the pitch P-controller. #4.0
pid_i_gain_yaw = 0.002;               #Gain setting for the pitch I-controller. #0.02
pid_d_gain_yaw = 0.0;                 #Gain setting for the pitch D-controller.
pid_max_yaw = 400;                    #Maximum output of the PID-controller (+/-)

deltaP = 0.1
deltaI = 0.0001
deltaD = 0.1
# ============================= Functions ==================================
# Function to convert
def listToString(s):

    # initialize an empty string
    str1 = ""

    # traverse in the string
    for ele in s:
        str1 += ele

    # return string
    return str1

def sndUART(ser,send_message):
    send_lim = "$"
    send_end = "---"
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

# sample thread
def print_time( threadName, delay):
   count = 0
   while count < 5:
      time.sleep(delay)
      count += 1
      print ("%s: %s" % ( threadName, time.ctime(time.time()) ))

def get_keyboard(delay):
    global pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll
    global pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch
    global pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw

    count = 0
    while 1:
       # Show commands
       if(count==0):
           print("==== COMANDS ===")
           print("---> Roll <---")
           print("Q/A: increase/decrease gain P")
           print("W/S: increase/decrease gain I")
           print("E/D: increase/decrease gain D")
           print("---> PItch <---")
           print("R/F: increase/decrease gain P")
           print("T/G: increase/decrease gain I")
           print("Y/H: increase/decrease gain D")
           print("---> Yaw <---")
           print("U/J: increase/decrease gain P")
           print("I/K: increase/decrease gain I")
           print("O/L: increase/decrease gain D")
           print("==============")

       keyPressed = raw_input()
       #print(keyPressed)
       # roll
       if (keyPressed.upper()=='Q'):
           pid_p_gain_roll += deltaP
       elif(keyPressed.upper()=='A'):
           pid_p_gain_roll -= deltaP

       elif (keyPressed.upper()=='W'):
           pid_i_gain_roll += deltaI
       elif(keyPressed.upper()=='S'):
           pid_i_gain_roll -= deltaI

       elif (keyPressed.upper()=='E'):
           pid_d_gain_roll += deltaD
       elif(keyPressed.upper()=='D'):
           pid_d_gain_roll -= deltaD

       # pitch
       if (keyPressed.upper()=='R'):
           pid_p_gain_pitch += deltaP
       elif(keyPressed.upper()=='F'):
           pid_p_gain_pitch -= deltaP

       elif (keyPressed.upper()=='T'):
           pid_i_gain_pitch += deltaI
       elif(keyPressed.upper()=='G'):
           pid_i_gain_pitch -= deltaI

       elif (keyPressed.upper()=='Y'):
           pid_d_gain_pitch += deltaD
       elif(keyPressed.upper()=='H'):
           pid_d_gain_pitch -= deltaD

       # yaw
       if (keyPressed.upper()=='U'):
           pid_p_gain_yaw += deltaP
       elif(keyPressed.upper()=='J'):
           pid_p_gain_yaw -= deltaP

       elif (keyPressed.upper()=='I'):
           pid_i_gain_yaw += deltaI
       elif(keyPressed.upper()=='K'):
           pid_i_gain_yaw -= deltaI

       elif (keyPressed.upper()=='O'):
           pid_d_gain_yaw += deltaD
       elif(keyPressed.upper()=='L'):
           pid_d_gain_yaw -= deltaD

       count += 1
       if(count>=7):
           count = 0

       print("Roll P=",str(pid_p_gain_roll),", I=",str(pid_i_gain_roll),", D=",str(pid_d_gain_roll))
       print("Pitch P=",str(pid_p_gain_pitch),", I=",str(pid_i_gain_pitch),", D=",str(pid_d_gain_pitch))
       print("Yaw P=",str(pid_p_gain_yaw),", I=",str(pid_i_gain_yaw),", D=",str(pid_d_gain_yaw))

       time.sleep(delay)


# ============================= MAIN ==================================
print('Hello world!')
serUSB = serial.Serial(port = "/dev/ttyUSB1", baudrate=115200, bytesize=8, timeout=0.2, stopbits=serial.STOPBITS_ONE)
serUSB.reset_output_buffer()
serUSB.reset_input_buffer()
# serUSB.flushOutput()      # Python 3.0
# serUSB.flushInput()       # Python 3.0

j = 0
from_patmos = ""
to_patmos = "hello world"

try:
    # Start keyboard parallel
    thread.start_new_thread(get_keyboard, (0.1,))
except:
   print ("Error: unable to start thread")

while 1:
    # ========== RECEIVE ==========
    #from_patmos = rcvUART(serUSB)
    #show = "Message: "+ from_patmos
    #print(show)
    # decode
    #data_ls = from_patmos.split(";")
    #if(data_ls[0] == 'DATA'):
    
    # ========== SEND ==========
    global pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll
    global pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch
    global pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw
    # roll
    to_patmos = ''
    if(pid_p_gain_roll<1):
        to_patmos += '0'
    to_patmos += str(int(pid_p_gain_roll*10))+';'

    if(pid_i_gain_roll<0.001):
        to_patmos += '0'
    to_patmos += str(int(pid_i_gain_roll*10000))+';'+str(int(pid_d_gain_roll*100))+';'
    
    # pitch
    if(pid_p_gain_pitch<1):
        to_patmos += '0'
    to_patmos += str(int(pid_p_gain_pitch*10))+';'
    if(pid_i_gain_pitch<0.001):
        to_patmos += '0'
    to_patmos += str(int(pid_i_gain_pitch*10000))+';'+str(int(pid_d_gain_pitch*100))+';'

    # yaw
    if(pid_p_gain_yaw<1):
        to_patmos += '0'
    to_patmos += str(int(pid_p_gain_yaw*10))+';'
    if(pid_i_gain_yaw<0.001):
        to_patmos += '0'
    to_patmos += str(int(pid_i_gain_yaw*10000))+';'+'0'+'0'+str(int(pid_d_gain_yaw*100))+';'

    sndUART(serUSB,to_patmos)
    # see message that is been sent to the FPGA
    #print(to_patmos)
    to_patmos = ""

    time.sleep(0.5)
    j+=1