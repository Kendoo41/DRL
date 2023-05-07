import socket
import time
import threading
import codecs


# Params for NX100 controller
nx100Address = "169.254.198.23"
nx100tcpPort = 80

# Initialize serial comm port for communicating with extruder controller
CR = "\r"
CRLF = "\r\n"

def utf8len(inputString):
    return len(inputString.encode('utf-8'))

def command_data_length(command):
    if len(command) is 0:
        return 0
    else:
        return utf8len(command + CR)

def robotComm():

    for command in hexFile:
        # a = time.clock()
        command = command.rstrip()
        #Comm setup
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)
        
        #Connect to the client/NX100 controller
        client.connect((nx100Address, nx100tcpPort))

        #START request
        startRequest = "CONNECT Robot_access" + CRLF
        client.send(startRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096) #4096: buffer size
        startResponse = repr(response)
        # print(startResponse)
        if 'OK: NX Information Server' not in startResponse:
            client.close()
            print('[E] Command start request response to NX100 is not successful!')
            return
        
        #COMMAND request
        commandLength = command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST " + "MOVL" + " " + str(commandLength) + CRLF
        client.send(commandRequest.encode())
        time.sleep(0.01)
        response = client.recv(4096) #4096: buffer size
        commandResponse = repr(response)
        # print(commandResponse)
        if ('OK: ' + "MOVL" not in commandResponse):
            client.close()
            print('[E] Command request response to NX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + (CR if len(command) > 0 else '')
            client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = client.recv(4096)
            commandDataResponse = repr(response)
            # print(commandDataResponse)
            if commandDataResponse:
                #Close socket
                client.close()
        time.sleep(0.05)
        # b = time.clock()
        # print(b-a)

# robotComm()

def extruderComm():
    if serArd.isOpen():
        print("Hot end heated!")
        extFile = open(r"F:\Ho Chi Minh University of Technology\Sem 8\robotPath\75mmpsEXT.txt", "r")
        startChar = "S" + "\0"
        time.sleep(1)
        for command in extFile:
            # a = time.clock()
            recData = 0
            serArd.write(startChar.encode())
            time.sleep(0.01)
            while (recData != 68):
                while serArd.in_waiting:
                    recData = int.from_bytes(serArd.readline(serArd.in_waiting), byteorder = 'big', signed = False)
                    if (recData == 82):    #recData == 65 for running stepper motor only
                        serArd.write((codecs.decode(command, 'unicode_escape') + "\0").encode())
                    else: time.sleep(0.01)  
            time.sleep(0.01)
            # b = time.clock()
            # print(b-a)

if __name__ == '__main__':
    runFlag = 0
    if serArd.isOpen():
        print("Connection is established to " + serArd.portstr)
        time.sleep(1)                                                                           
        while (runFlag != 1):
            tempData = 0
            startChar = "S" + "\0"                          #Send "S"
            serArd.write(startChar.encode())                                                      
            time.sleep(0.01)
            while serArd.in_waiting:
                tempData = int.from_bytes(serArd.readline(serArd.in_waiting), byteorder = 'big', signed = False)            
                if (tempData == 65):                        #Receive "A"
                    print("Heating...") 
                if (tempData == 82):                        #Receive "R"
                    runFlag = 1
    if (runFlag == 1):    
        time.sleep(1)
        thread1 = threading.Thread(target = robotComm)
        thread1.start()
        thread2 = threading.Thread(target = extruderComm)
        thread2.start()
