import serial
import time
# ============================= Variables ==================================
# message = [];
# temp = [];
# start = False
# start_i = 1
# start_check = False
# start_lim = ['.','F','P','G','A','1'];
# start_lim_n = len(start_lim)
#
# end_lim = "!";
# #end_lim_n = len(end_lim)
#
# dummy = False
# i = 0

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

while j<100:
    # receive
    from_patmos = rcvUART(serUSB)
    show = "Message: "+ from_patmos
    print(show)
    # send
    sndUART(serUSB,to_patmos)

    time.sleep(0.1)
    j+=1
