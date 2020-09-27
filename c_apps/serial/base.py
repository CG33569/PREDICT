import serial
import time
# ============================= Variables ==================================
message = [];
temp = [];
start = False
start_i = 1
start_check = False
start_lim = ['$','P','R','E','D','I'];
start_lim_n = len(start_lim)

end_lim = ['%'];
end_lim_n = len(end_lim)

dummy = False
i = 0
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

# ============================= MAIN ==================================
print('Hello world!')
ser = serial.Serial(port = "/dev/ttyUSB2", baudrate=9600, bytesize=8, timeout=0.2, stopbits=serial.STOPBITS_ONE)
while i<100:
    print("--> Sending a message <--")
    message_to_board = 'Hello Patmos!\n\r'
    ser.write(message_to_board.encode('utf-8'))
    i+=1

i = 0    
while 1:
    data = ser.read()                         #read byte from serial device
    data = data.decode('utf-8')              #decode message

    if (data=="$")and(not start):
        start = True
        print("Appending\n")

    if (data =="!") and (start_check):
        print("--> Message: <--")
        print(listToString(message))
        # Analyse message here:
        print("\n--> End of message nr.",i,"<--")
        # Send message at the end:
        print("--> Sending a message <--")
        message_to_board = 'Hello Patmos!\n\r'
        ser.write(message_to_board.encode('utf-8'))
        message_to_board=''
        message.clear()
        temp.clear()
        start_check = False
        start = False
        dummy = False
        i+=1


    if start_check:
        message.append(data)
        #print(listToString(message))
        if (not dummy):
            print("Saving message.")
            dummy = True

    if start:
        temp.append(data)
        start_i+=1
        #check beginning
        if (len(temp)==start_lim_n)and(not start_check):
            start_check = (temp == start_lim)
            print(temp)
            print("Found start")
            print("Start check ",start_check)
            start = False
            temp.clear()
